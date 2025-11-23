import { Component, ChangeDetectionStrategy, ElementRef, ViewChild, AfterViewInit, OnDestroy, input, output, effect, WritableSignal, Signal, computed } from '@angular/core';
import { PhysicsObject, DynamicObject, Plane, Conveyor, Spring, Ball, ObjectType, Arc, Block, Rod, Pin, GraphDataPoint, FieldRegion } from '../../models/physics-objects.model';
import { Vector2D } from '../../models/vector.model';

const PIN_CLICK_RADIUS = 10;
const SNAP_DISTANCE = 15;
const CLICK_TOLERANCE = 8;
const WEAK_SNAP_DISTANCE = 5;

@Component({
  selector: 'app-simulation-canvas',
  templateUrl: './simulation-canvas.component.html',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class SimulationCanvasComponent implements AfterViewInit, OnDestroy {
  @ViewChild('physicsCanvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;
  
  objects = input.required<WritableSignal<PhysicsObject[]>>();
  settings = input.required<{ isRunning: boolean; gravity: number; collisionsEnabled: boolean }>();
  placingObjectType = input.required<Signal<ObjectType | null>>();
  selectedObject = input<Signal<PhysicsObject | null>>();
  
  selectObject = output<number | null>();
  placeObject = output<any>();
  dataPoint = output<GraphDataPoint>();

  private ctx!: CanvasRenderingContext2D;
  private animationFrameId: number | null = null;
  private lastTime: number = 0;
  private simulationTime = 0;

  private zoom = 1.0;
  private panOffset = new Vector2D(0, 0);
  private isPanning = false;
  private lastPanPosition = new Vector2D(0, 0);

  private isDragging = false;
  private dragTarget: (Ball | Block | FieldRegion) | null = null;
  private dragOffset = new Vector2D(0, 0);
  private mousePosWorld = new Vector2D(0, 0);
  private snappedObject: PhysicsObject | null = null;
  private hoveredObject: PhysicsObject | null = null;

  private placementState: {
    isPlacing: boolean;
    type: ObjectType | null;
    step: 'start' | 'end' | 'center' | 'radius_start' | 'end_angle';
    startPoint?: Vector2D;
    centerPoint?: Vector2D;
    radius?: number;
    startAngle?: number;
    object1Id?: number;
  } = { isPlacing: false, type: null, step: 'start' };

  private objectsMap = computed(() => new Map(this.objects()().map(o => [o.id, o])));
  
  constructor() {
    effect(() => {
      const isRunning = this.settings().isRunning;
      if (isRunning && !this.animationFrameId) {
        this.lastTime = performance.now();
        this.gameLoop();
      } else if (!isRunning && this.animationFrameId) {
        cancelAnimationFrame(this.animationFrameId);
        this.animationFrameId = null;
        this.draw();
      }
    });

    effect(() => {
      this.objects();
      if (!this.settings().isRunning) {
        this.draw();
      }
    });
    
    effect(() => {
      const type = this.placingObjectType()?.();
      const step = type === 'arc' ? 'center' : 'start';
      this.placementState = type ? { isPlacing: true, type, step } : { isPlacing: false, type: null, step: 'start' };
      this.updateCursor();
    });
  }

  ngAfterViewInit(): void {
    this.ctx = this.canvasRef.nativeElement.getContext('2d')!;
    this.resizeCanvas();
    window.addEventListener('resize', this.resizeCanvas);
    const canvas = this.canvasRef.nativeElement;
    canvas.addEventListener('mousedown', this.onMouseDown);
    canvas.addEventListener('mousemove', this.onMouseMove);
    canvas.addEventListener('mouseup', this.onMouseUp);
    canvas.addEventListener('mouseleave', this.onMouseLeave);
    canvas.addEventListener('wheel', this.onWheel, { passive: false });
    this.draw();
    setTimeout(() => this.resizeCanvas(), 0);
  }

  ngOnDestroy(): void {
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
    }
    window.removeEventListener('resize', this.resizeCanvas);
  }

  // --- Public Methods for Parent Component ---
  public zoomIn(): void { this.applyZoom(1.25); }
  public zoomOut(): void { this.applyZoom(0.8); }
  public resetView(): void {
    this.zoom = 1.0;
    this.panOffset = new Vector2D(0, 0);
    this.draw();
  }
  public exportAsImage(): void {
    const canvas = this.canvasRef.nativeElement;
    const link = document.createElement('a');
    link.download = `physics-sim-${new Date().toISOString().slice(0,10)}.png`;
    link.href = canvas.toDataURL('image/png');
    link.click();
  }
  public resetTime(): void {
    this.simulationTime = 0;
  }

  // --- Physics Loop ---
  private gameLoop = (currentTime: number = performance.now()): void => {
    const deltaTime = (currentTime - this.lastTime) / 1000; // in seconds
    this.lastTime = currentTime;
    this.update(Math.min(deltaTime, 0.1));
    this.draw();
    this.animationFrameId = requestAnimationFrame(this.gameLoop);
  }

  private update(dt: number): void {
    this.simulationTime += dt;
    const selected = this.selectedObject?.()?.();
    if (selected && (selected.type === 'ball' || selected.type === 'block')) {
        this.dataPoint.emit({
            t: this.simulationTime,
            x: selected.position.x,
            y: selected.position.y,
            vx: selected.velocity.x,
            vy: selected.velocity.y,
        });
    }

    const objects = this.objects()();
    const dynamicObjects = objects.filter((o): o is Ball | Block => o.type === 'ball' || o.type === 'block');

    for (const obj of dynamicObjects) {
      if (obj === this.dragTarget) continue;
      obj.force = new Vector2D(0, obj.mass * this.settings().gravity * 20);
      if (obj.type === 'block') obj.torque = 0;
    }
      
    this.applySpringForces();
    this.applyElectricForces(dynamicObjects);
    this.applyExternalFieldForces(objects);

    for (const obj of dynamicObjects) {
        if (obj === this.dragTarget) continue;
        obj.acceleration = obj.force.multiply(1 / obj.mass);
        obj.velocity = obj.velocity.add(obj.acceleration.multiply(dt));
        obj.position = obj.position.add(obj.velocity.multiply(dt));
        if (obj.type === 'block' && obj.momentOfInertia > 0) {
            const angularAcceleration = obj.torque / obj.momentOfInertia;
            obj.angularVelocity += angularAcceleration * dt;
            obj.angle += obj.angularVelocity * dt;
        }
    }

    this.applyConstraints(5);

    if (this.settings().collisionsEnabled) {
        this.handleCollisions();
    }
  }

  private applySpringForces(): void {
    const springs = this.objects()().filter((s): s is Spring => s.type === 'spring');
    const objectMap = this.objectsMap();
    for (const spring of springs) {
        const obj1 = objectMap.get(spring.object1Id);
        const obj2 = objectMap.get(spring.object2Id);
        if (obj1 && obj2 && 'position' in obj1 && 'position' in obj2) {
            const p1 = obj1.position; const p2 = obj2.position;
            const springVec = p2.subtract(p1);
            const length = springVec.magnitude();
            if (length === 0) continue;

            const displacement = length - spring.restLength;
            const springForceMag = spring.stiffness * displacement;
            const springForce = springVec.normalize().multiply(springForceMag);
            
            if ('force' in obj1) (obj1 as DynamicObject).force = (obj1 as DynamicObject).force.add(springForce);
            if ('force' in obj2) (obj2 as DynamicObject).force = (obj2 as DynamicObject).force.subtract(springForce);
        }
    }
  }

  private applyElectricForces(dynamicObjects: DynamicObject[]): void {
    const k = 20000; // Scaled Coulomb's constant
    for (let i = 0; i < dynamicObjects.length; i++) {
        const obj1 = dynamicObjects[i];
        if (!obj1.charge) continue;

        for (let j = i + 1; j < dynamicObjects.length; j++) {
            const obj2 = dynamicObjects[j];
            if (!obj2.charge) continue;

            const vec = obj1.position.subtract(obj2.position);
            let distSq = vec.x * vec.x + vec.y * vec.y;
            
            // Avoid singularity and huge forces at very close distances
            distSq = Math.max(distSq, 400); // Min distance of 20px

            const forceMag = k * (obj1.charge * obj2.charge) / distSq;
            const forceVec = vec.normalize().multiply(forceMag);

            obj1.force = obj1.force.add(forceVec);
            obj2.force = obj2.force.subtract(forceVec);
        }
    }
  }

  private applyExternalFieldForces(objects: PhysicsObject[]): void {
      const dynamicObjects = objects.filter((o): o is Ball | Block => o.type === 'ball' || o.type === 'block');
      const fieldRegions = objects.filter((f): f is FieldRegion => f.type === 'field');

      for (const obj of dynamicObjects) {
          if (!obj.charge) continue;

          for (const region of fieldRegions) {
              // Transform object position to field's local coordinates to handle rotation
              const localPos = obj.position.subtract(region.position);
              const cos = Math.cos(-region.angle);
              const sin = Math.sin(-region.angle);
              const rotatedPos = new Vector2D(
                  localPos.x * cos - localPos.y * sin,
                  localPos.x * sin + localPos.y * cos
              );

              const isInside = Math.abs(rotatedPos.x) < region.width / 2 && Math.abs(rotatedPos.y) < region.height / 2;

              if (isInside) {
                  // Electric force: F = qE. E is in local space, so rotate force back to world space.
                  const localElectricForce = region.electricField;
                  const worldCos = Math.cos(region.angle);
                  const worldSin = Math.sin(region.angle);
                  const electricForce = new Vector2D(
                      localElectricForce.x * worldCos - localElectricForce.y * worldSin,
                      localElectricForce.x * worldSin + localElectricForce.y * worldCos
                  ).multiply(obj.charge);
                  obj.force = obj.force.add(electricForce);

                  // Magnetic force: F = q(v x B). In 2D, with B in z-dir: F = q * (vy*Bz, -vx*Bz).
                  // This force is dependent on world velocity and is correctly calculated in world space.
                  if (region.magneticField !== 0) {
                      const magneticForce = new Vector2D(
                          obj.velocity.y * region.magneticField,
                          -obj.velocity.x * region.magneticField
                      ).multiply(obj.charge);
                      obj.force = obj.force.add(magneticForce);
                  }
              }
          }
      }
  }

  private applyConstraints(iterations: number): void {
    const rods = this.objects()().filter((r): r is Rod => r.type === 'rod');
    const objectMap = this.objectsMap();
    for (let i = 0; i < iterations; i++) {
        for (const rod of rods) {
            const obj1 = objectMap.get(rod.object1Id);
            const obj2 = objectMap.get(rod.object2Id);
            if (!obj1 || !obj2 || !('position' in obj1) || !('position' in obj2)) continue;
            
            const pos1 = obj1.position; const pos2 = obj2.position;
            const axis = pos1.subtract(pos2);
            const dist = axis.magnitude();
            if (dist === 0) continue;

            const diff = (dist - rod.length) / dist;
            const mass1 = ('mass' in obj1) ? (obj1 as DynamicObject).mass : Infinity;
            const mass2 = ('mass' in obj2) ? (obj2 as DynamicObject).mass : Infinity;
            const totalInverseMass = (1 / mass1) + (1 / mass2);
            if (totalInverseMass === 0) continue;

            const correction = axis.multiply(diff);

            if (isFinite(mass1)) (obj1 as DynamicObject).position = pos1.subtract(correction.multiply((1 / mass1) / totalInverseMass));
            if (isFinite(mass2)) (obj2 as DynamicObject).position = pos2.add(correction.multiply((1 / mass2) / totalInverseMass));

            // Velocity correction to prevent energy gain
            const v1 = isFinite(mass1) ? (obj1 as DynamicObject).velocity : new Vector2D(0, 0);
            const v2 = isFinite(mass2) ? (obj2 as DynamicObject).velocity : new Vector2D(0, 0);
            const relVel = v1.subtract(v2);
            const velAlongAxis = relVel.dot(axis.normalize());
            
            if (velAlongAxis === 0) continue;

            const impulseMag = -velAlongAxis / totalInverseMass;
            const impulse = axis.normalize().multiply(impulseMag);

            if (isFinite(mass1)) (obj1 as DynamicObject).velocity = (obj1 as DynamicObject).velocity.add(impulse.multiply(1 / mass1));
            if (isFinite(mass2)) (obj2 as DynamicObject).velocity = (obj2 as DynamicObject).velocity.subtract(impulse.multiply(1 / mass2));
        }
    }
  }

  // --- Collision Handling ---
  private handleCollisions(): void {
    const objects = this.objects()();
    const dynamicObjects = objects.filter((o): o is Ball | Block => o.type === 'ball' || o.type === 'block');

    for (let i = 0; i < dynamicObjects.length; i++) {
        for (let j = i + 1; j < dynamicObjects.length; j++) {
            this.resolveDynamicCollision(dynamicObjects[i], dynamicObjects[j]);
        }
    }

    for (const dObj of dynamicObjects) {
        for (const sObj of objects) {
            if (dObj.id === sObj.id || sObj.type === 'ball' || sObj.type === 'block' || sObj.type === 'field') continue;
            this.resolveStaticCollision(dObj, sObj);
        }
    }
  }

  private resolveDynamicCollision(objA: Ball | Block, objB: Ball | Block): void {
      if (objA.type === 'ball' && objB.type === 'ball') {
        // Not implemented as it wasn't requested, but simple to add if needed
      } else if (objA.type === 'ball' && objB.type === 'block') {
          this.handleBallBlockCollision(objA, objB);
      } else if (objA.type === 'block' && objB.type === 'ball') {
          this.handleBallBlockCollision(objB, objA);
      } else if (objA.type === 'block' && objB.type === 'block') {
          this.handleBlockBlockCollision(objA, objB);
      }
  }

  private resolveStaticCollision(dObj: Ball | Block, sObj: PhysicsObject): void {
      if (sObj.type === 'plane' || sObj.type === 'conveyor') {
        this.handlePlaneCollision(dObj, sObj);
      } else if (sObj.type === 'arc') {
          this.handleArcCollision(dObj, sObj);
      }
  }
  
  private handlePlaneCollision(obj: Ball | Block, plane: Plane | Conveyor): void {
    if (obj.type === 'ball') {
        this.handleBallPlaneCollision(obj, plane);
    } else if (obj.type === 'block') {
        this.handleBlockPlaneCollision(obj, plane);
    }
  }

  private handleBallPlaneCollision(ball: Ball, plane: Plane | Conveyor): void {
    const lineVec = plane.end.subtract(plane.start);
    const lineLenSq = lineVec.x * lineVec.x + lineVec.y * lineVec.y;

    if (lineLenSq === 0) return;

    // Find the closest point on the line segment to the ball's center
    const t = Math.max(0, Math.min(1, ball.position.subtract(plane.start).dot(lineVec) / lineLenSq));
    const closestPointOnSegment = plane.start.add(lineVec.multiply(t));

    const toBall = ball.position.subtract(closestPointOnSegment);
    const dist = toBall.magnitude();

    if (dist < ball.radius) {
      const penetration = ball.radius - dist;
      // The normal is from the closest point on the segment to the ball's center
      const normal = dist > 0 ? toBall.normalize() : new Vector2D(-lineVec.y, lineVec.x).normalize();
      
      ball.position = ball.position.add(normal.multiply(penetration * 1.01));
      this.applyImpulse(ball, null, normal, plane.restitution, plane.friction);
      
      if (plane.type === 'conveyor') {
          const conveyorVel = lineVec.normalize().multiply(plane.speed);
          const conveyorForce = conveyorVel.subtract(ball.velocity).multiply(plane.friction * 50);
          ball.force = ball.force.add(conveyorForce);
      }
    }
  }

  private handleBlockPlaneCollision(block: Block, plane: Plane | Conveyor): void {
    const vertices = this.getBlockVertices(block);
    const lineVec = plane.end.subtract(plane.start);
    if (lineVec.magnitude() === 0) return;

    let normal = new Vector2D(-lineVec.y, lineVec.x).normalize();
    // Ensure normal points "out" of the plane towards the block, making it independent of draw direction
    if (block.position.subtract(plane.start).dot(normal) < 0) {
        normal = normal.multiply(-1);
    }

    const lineDir = lineVec.normalize();
    const lineLength = lineVec.magnitude();

    let maxPenetration = 0;
    let contactVertex: Vector2D | null = null;

    for (const vertex of vertices) {
        const projection = vertex.subtract(plane.start).dot(lineDir);
        if (projection >= 0 && projection <= lineLength) {
            const dist = vertex.subtract(plane.start).dot(normal);
            if (dist < 0) {
                const penetration = -dist;
                if (penetration > maxPenetration) {
                    maxPenetration = penetration;
                    contactVertex = vertex;
                }
            }
        }
    }

    if (contactVertex) {
        block.position = block.position.add(normal.multiply(maxPenetration * 1.01));
        this.applyImpulse(block, null, normal, plane.restitution, plane.friction, contactVertex);
    }
  }
  
  private handleArcCollision(obj: Ball | Block, arc: Arc): void {
    if (obj.type !== 'ball') return;
    const toBall = obj.position.subtract(arc.center);
    const dist = toBall.magnitude();
    if (dist === 0) return;

    let angle = Math.atan2(toBall.y, toBall.x);
    if (angle < 0) angle += 2 * Math.PI;
    
    const start = arc.startAngle; const end = arc.endAngle;
    const inAngleRange = (start < end) ? (angle >= start && angle <= end) : (angle >= start || angle <= end);

    if (!inAngleRange) return;

    const penetration = obj.radius - Math.abs(dist - arc.radius);
    if (penetration > 0) {
        // Normal points from center to ball for convex (outside) collision,
        // and from ball to center for concave (inside) collision.
        const normal = dist > arc.radius ? toBall.normalize() : toBall.normalize().multiply(-1);
        const relativeVelocity = obj.velocity; // Arc is static
        if (relativeVelocity.dot(normal) < 0) {
            obj.position = obj.position.add(normal.multiply(penetration * 1.01));
            this.applyImpulse(obj, null, normal, arc.restitution, arc.friction);
        }
    }
  }

    private handleBallBlockCollision(ball: Ball, block: Block): void {
        const vertices = this.getBlockVertices(block);
        let closestPoint = this.findClosestVertexToPoint(vertices, ball.position);
        
        const axes = this.getBlockAxes(vertices);
        axes.push(closestPoint.subtract(ball.position).normalize());

        let minOverlap = Infinity;
        let mtv: Vector2D | null = null;
        
        for(const axis of axes) {
            const ballRange = this.projectBall(ball, axis);
            const blockRange = this.projectVertices(vertices, axis);

            const overlap = Math.min(ballRange.max, blockRange.max) - Math.max(ballRange.min, blockRange.min);
            if (overlap < 0) return; // No collision

            if (overlap < minOverlap) {
                minOverlap = overlap;
                mtv = axis;
            }
        }
        if (!mtv) return;

        const centerVec = block.position.subtract(ball.position);
        if (centerVec.dot(mtv) < 0) {
            mtv = mtv.multiply(-1);
        }

        ball.position = ball.position.subtract(mtv.multiply(minOverlap * 0.51));
        block.position = block.position.add(mtv.multiply(minOverlap * 0.51));
        
        this.applyDynamicImpulse(ball, block, mtv, closestPoint);
    }

    private handleBlockBlockCollision(blockA: Block, blockB: Block): void {
        const verticesA = this.getBlockVertices(blockA);
        const verticesB = this.getBlockVertices(blockB);
        const axes = [...this.getBlockAxes(verticesA), ...this.getBlockAxes(verticesB)];

        let minOverlap = Infinity;
        let mtv: Vector2D | null = null;

        for (const axis of axes) {
            const rangeA = this.projectVertices(verticesA, axis);
            const rangeB = this.projectVertices(verticesB, axis);
            const overlap = Math.min(rangeA.max, rangeB.max) - Math.max(rangeA.min, rangeB.min);
            if (overlap < 0) return;
            if (overlap < minOverlap) {
                minOverlap = overlap;
                mtv = axis;
            }
        }

        if (!mtv) return;
        
        const centerVec = blockB.position.subtract(blockA.position);
        if (centerVec.dot(mtv) < 0) {
            mtv = mtv.multiply(-1);
        }

        const invMassA = 1 / blockA.mass;
        const invMassB = 1 / blockB.mass;
        const totalInvMass = invMassA + invMassB;
        if (totalInvMass === 0) return;
        
        const penetration = mtv.multiply(minOverlap + 0.01);
        blockA.position = blockA.position.subtract(penetration.multiply(invMassA / totalInvMass));
        blockB.position = blockB.position.add(penetration.multiply(invMassB / totalInvMass));
        
        this.applyDynamicImpulse(blockA, blockB, mtv);
    }

    private applyDynamicImpulse(objA: Ball | Block, objB: Ball | Block, normal: Vector2D, contactPoint?: Vector2D): void {
        const restitution = Math.min(objA.restitution, objB.restitution);
        const friction = Math.sqrt(objA.friction * objB.friction);
        
        contactPoint = contactPoint || this.findContactPoint(objA, objB);
        
        const rA = contactPoint.subtract(objA.position);
        const rB = contactPoint.subtract(objB.position);
        
        const angVelA = (objA as Block).angularVelocity || 0;
        const angVelB = (objB as Block).angularVelocity || 0;
        
        const vA = objA.velocity.add(new Vector2D(-angVelA * rA.y, angVelA * rA.x));
        const vB = objB.velocity.add(new Vector2D(-angVelB * rB.y, angVelB * rB.x));
        const vRel = vA.subtract(vB);

        const vDotN = vRel.dot(normal);
        if (vDotN >= 0) return;

        const invMassA = 1 / objA.mass;
        const invMassB = 1 / objB.mass;
        const invInertiaA = objA.type === 'block' ? 1 / objA.momentOfInertia : 0;
        const invInertiaB = objB.type === 'block' ? 1 / objB.momentOfInertia : 0;
        
        const rACrossN = rA.x * normal.y - rA.y * normal.x;
        const rBCrossN = rB.x * normal.y - rB.y * normal.x;

        const impulseDenominator = invMassA + invMassB + (rACrossN * rACrossN) * invInertiaA + (rBCrossN * rBCrossN) * invInertiaB;
        if (impulseDenominator === 0) return;
        
        // Normal Impulse
        const jN = -(1 + restitution) * vDotN / impulseDenominator;
        const impulseN = normal.multiply(jN);

        objA.velocity = objA.velocity.add(impulseN.multiply(invMassA));
        objB.velocity = objB.velocity.subtract(impulseN.multiply(invMassB));
        if (objA.type === 'block') objA.angularVelocity += (rA.x * impulseN.y - rA.y * impulseN.x) * invInertiaA;
        if (objB.type === 'block') objB.angularVelocity -= (rB.x * impulseN.y - rB.y * impulseN.x) * invInertiaB;

        // Friction Impulse
        const tangent = vRel.subtract(normal.multiply(vDotN)).normalize();
        if (tangent.magnitude() === 0) return;

        const vDotT = vRel.dot(tangent);
        
        const rACrossT = rA.x * tangent.y - rA.y * tangent.x;
        const rBCrossT = rB.x * tangent.y - rB.y * tangent.x;
        
        const frictionDenominator = invMassA + invMassB + (rACrossT * rACrossT) * invInertiaA + (rBCrossT * rBCrossT) * invInertiaB;
        if (frictionDenominator === 0) return;

        let jT = -vDotT / frictionDenominator;
        
        // Clamp friction impulse
        if (Math.abs(jT) > jN * friction) {
            jT = jN * friction * Math.sign(jT);
        }
        
        const impulseT = tangent.multiply(jT);
        
        objA.velocity = objA.velocity.add(impulseT.multiply(invMassA));
        objB.velocity = objB.velocity.subtract(impulseT.multiply(invMassB));
        if (objA.type === 'block') objA.angularVelocity += (rA.x * impulseT.y - rA.y * impulseT.x) * invInertiaA;
        if (objB.type === 'block') objB.angularVelocity -= (rB.x * impulseT.y - rB.y * impulseT.x) * invInertiaB;
    }
  
  private applyImpulse(objA: Ball | Block, objB: DynamicObject | null, normal: Vector2D, restitution: number, friction: number, contactPoint?: Vector2D): void {
    contactPoint = contactPoint || objA.position;
    const rA = contactPoint.subtract(objA.position);
    
    const angularVelocityA = objA.type === 'block' ? objA.angularVelocity : 0;
    const vA = objA.velocity.add(new Vector2D(-angularVelocityA * rA.y, angularVelocityA * rA.x));
    const vB = new Vector2D(0,0); // Assuming static for now
    const vRel = vA.subtract(vB);

    let vDotN = vRel.dot(normal);
    if (vDotN >= 0) return;

    const e = restitution;
    const invMassA = 1/objA.mass;
    const invInertiaA = objA.type === 'block' ? 1/objA.momentOfInertia : 0;

    const jN = -(1 + e) * vDotN / (invMassA + Math.pow(rA.x * normal.y - rA.y * normal.x, 2) * invInertiaA);
    const impulseN = normal.multiply(jN);

    objA.velocity = objA.velocity.add(impulseN.multiply(invMassA));
    if (objA.type === 'block') {
        objA.angularVelocity += (rA.x * impulseN.y - rA.y * impulseN.x) * invInertiaA;
    }

    // Friction impulse
    const tangent = vRel.subtract(normal.multiply(vDotN)).normalize();
    if(tangent.magnitude() === 0) return;

    const vDotT = vRel.dot(tangent);
    
    const jT = -vDotT / (invMassA + Math.pow(rA.x * tangent.y - rA.y * tangent.x, 2) * invInertiaA);
    const frictionImpulse = tangent.multiply(Math.max(-friction * jN, Math.min(friction * jN, jT)));

    objA.velocity = objA.velocity.add(frictionImpulse.multiply(invMassA));
    if (objA.type === 'block') {
        objA.angularVelocity += (rA.x * frictionImpulse.y - rA.y * frictionImpulse.x) * invInertiaA;
    }
}


  // --- Drawing ---
  private draw(): void {
    if (!this.ctx) return;
    const { width, height } = this.canvasRef.nativeElement;
    this.ctx.clearRect(0, 0, width, height);

    this.ctx.save();
    this.ctx.translate(this.panOffset.x, this.panOffset.y);
    this.ctx.scale(this.zoom, this.zoom);

    this.drawGrid();

    for (const obj of this.objects()()) {
      this.ctx.save();
      if (obj.isSelected || obj === this.hoveredObject) {
        this.ctx.shadowColor = 'cyan';
        this.ctx.shadowBlur = 15 / this.zoom;
      }
      this.drawObject(obj);
      this.ctx.restore();
    }
    
    this.drawSnapIndicator();
    this.drawPlacementPreview();
    this.drawGhostObject();

    this.ctx.restore();
  }

  private drawGrid(): void {
    const { width, height } = this.canvasRef.nativeElement;
    const topLeft = this.getWorldPos(new Vector2D(0, 0));
    const bottomRight = this.getWorldPos(new Vector2D(width, height));

    let step = 100;
    if (this.zoom > 1.5) step = 50;
    if (this.zoom > 3) step = 25;
    if (this.zoom < 0.75) step = 200;
    if (this.zoom < 0.3) step = 500;

    this.ctx.strokeStyle = 'rgba(156, 163, 175, 0.2)'; // gray-400
    this.ctx.lineWidth = 1 / this.zoom;
    this.ctx.font = `${10 / this.zoom}px sans-serif`;
    this.ctx.fillStyle = 'rgba(156, 163, 175, 0.5)';

    // Vertical lines and labels
    const startX = Math.floor(topLeft.x / step) * step;
    for (let x = startX; x < bottomRight.x; x += step) {
      this.ctx.beginPath();
      this.ctx.moveTo(x, topLeft.y);
      this.ctx.lineTo(x, bottomRight.y);
      this.ctx.stroke();
      this.ctx.fillText(x.toFixed(0), x + 4 / this.zoom, topLeft.y + 12 / this.zoom);
    }

    // Horizontal lines and labels
    const startY = Math.floor(topLeft.y / step) * step;
    for (let y = startY; y < bottomRight.y; y += step) {
      this.ctx.beginPath();
      this.ctx.moveTo(topLeft.x, y);
      this.ctx.lineTo(bottomRight.x, y);
      this.ctx.stroke();
      if (y !== 0) { // Avoid label overlap with x-axis labels at origin
          this.ctx.fillText(y.toFixed(0), topLeft.x + 4 / this.zoom, y - 4 / this.zoom);
      }
    }

    // Axes
    this.ctx.strokeStyle = 'rgba(156, 163, 175, 0.4)';
    this.ctx.lineWidth = 1.5 / this.zoom;
    // Y-axis
    if (topLeft.x < 0 && bottomRight.x > 0) {
      this.ctx.beginPath();
      this.ctx.moveTo(0, topLeft.y);
      this.ctx.lineTo(0, bottomRight.y);
      this.ctx.stroke();
    }
    // X-axis
    if (topLeft.y < 0 && bottomRight.y > 0) {
      this.ctx.beginPath();
      this.ctx.moveTo(topLeft.x, 0);
      this.ctx.lineTo(bottomRight.x, 0);
      this.ctx.stroke();
    }
  }

  private drawObject(obj: PhysicsObject): void {
      switch (obj.type) {
        case 'ball': this.drawBall(obj); break;
        case 'block': this.drawBlock(obj); break;
        case 'plane': this.drawPlane(obj); break;
        case 'conveyor': this.drawConveyor(obj); break;
        case 'spring': this.drawSpring(obj); break;
        case 'arc': this.drawArc(obj); break;
        case 'rod': this.drawRod(obj); break;
        case 'pin': this.drawPin(obj); break;
        case 'field': this.drawFieldRegion(obj); break;
      }
  }

  private drawBall(obj: Ball): void {
    this.ctx.beginPath();
    this.ctx.arc(obj.position.x, obj.position.y, obj.radius, 0, Math.PI * 2);
    this.ctx.fillStyle = obj.color;
    this.ctx.fill();
    this.ctx.strokeStyle = '#fff';
    this.ctx.lineWidth = 2 / this.zoom;
    this.ctx.stroke();
  }

  private drawBlock(obj: Block): void {
     this.ctx.translate(obj.position.x, obj.position.y);
     this.ctx.rotate(obj.angle);
     this.ctx.fillStyle = obj.color;
     this.ctx.fillRect(-obj.width / 2, -obj.height / 2, obj.width, obj.height);
     this.ctx.strokeStyle = '#fff';
     this.ctx.lineWidth = 2 / this.zoom;
     this.ctx.strokeRect(-obj.width / 2, -obj.height / 2, obj.width, obj.height);
  }

  private drawPlane(obj: Plane): void {
    this.ctx.beginPath();
    this.ctx.moveTo(obj.start.x, obj.start.y);
    this.ctx.lineTo(obj.end.x, obj.end.y);
    this.ctx.strokeStyle = '#a0aec0';
    this.ctx.lineWidth = 5 / this.zoom;
    this.ctx.stroke();
  }
  
  private drawArc(obj: Arc): void {
    this.ctx.beginPath();
    this.ctx.arc(obj.center.x, obj.center.y, obj.radius, obj.startAngle, obj.endAngle);
    this.ctx.strokeStyle = '#a0aec0';
    this.ctx.lineWidth = 5 / this.zoom;
    this.ctx.stroke();
  }

  private drawConveyor(obj: Conveyor): void {
    this.ctx.beginPath();
    this.ctx.moveTo(obj.start.x, obj.start.y);
    this.ctx.lineTo(obj.end.x, obj.end.y);
    this.ctx.strokeStyle = '#63b3ed';
    this.ctx.lineWidth = 10 / this.zoom;
    this.ctx.stroke();
  }
  
  private drawSpring(obj: Spring): void {
    const p = this.getConnectorPoints(obj);
    if (!p) return;
    this.ctx.beginPath();
    this.ctx.moveTo(p.p1.x, p.p1.y);
    const dx = p.p2.x - p.p1.x, dy = p.p2.y - p.p1.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist === 0) return;
    const segments = 20;
    const coilWidth = 15;
    for (let i = 1; i < segments; i++) {
        const progress = i / segments;
        const offset = Math.sin(progress * Math.PI * 8) * coilWidth;
        this.ctx.lineTo(p.p1.x + dx * progress - dy / dist * offset, p.p1.y + dy * progress + dx / dist * offset);
    }
    this.ctx.lineTo(p.p2.x, p.p2.y);
    this.ctx.strokeStyle = '#f6e05e';
    this.ctx.lineWidth = 2 / this.zoom;
    this.ctx.stroke();
  }

  private drawRod(obj: Rod): void {
    const p = this.getConnectorPoints(obj);
    if (!p) return;
    this.ctx.beginPath();
    this.ctx.moveTo(p.p1.x, p.p1.y);
    this.ctx.lineTo(p.p2.x, p.p2.y);
    this.ctx.strokeStyle = '#e2e8f0';
    this.ctx.lineWidth = 6 / this.zoom;
    this.ctx.stroke();
  }
  
  private drawPin(obj: Pin): void {
    const r = PIN_CLICK_RADIUS / this.zoom;
    this.ctx.beginPath();
    this.ctx.arc(obj.position.x, obj.position.y, r - 2, 0, Math.PI * 2);
    this.ctx.fillStyle = '#4a5568';
    this.ctx.fill();
    this.ctx.moveTo(obj.position.x - r, obj.position.y); this.ctx.lineTo(obj.position.x + r, obj.position.y);
    this.ctx.moveTo(obj.position.x, obj.position.y - r); this.ctx.lineTo(obj.position.x, obj.position.y + r);
    this.ctx.strokeStyle = '#e2e8f0';
    this.ctx.lineWidth = 2 / this.zoom;
    this.ctx.stroke();
  }
  
  private drawFieldRegion(region: FieldRegion): void {
    this.ctx.translate(region.position.x, region.position.y);
    this.ctx.rotate(region.angle);
    
    // Draw semi-transparent background
    this.ctx.fillStyle = region.color;
    this.ctx.globalAlpha = 0.2;
    this.ctx.fillRect(-region.width / 2, -region.height / 2, region.width, region.height);
    this.ctx.globalAlpha = 1;

    // Draw border
    this.ctx.strokeStyle = region.color;
    this.ctx.lineWidth = 2 / this.zoom;
    this.ctx.strokeRect(-region.width / 2, -region.height / 2, region.width, region.height);

    const spacing = 40;
    // Draw Electric Field arrows
    if (region.electricField.magnitude() > 0.1) {
        this.ctx.strokeStyle = '#FBBF24'; // amber-400
        this.ctx.lineWidth = 1.5 / this.zoom;
        for (let x = -region.width / 2 + spacing/2; x < region.width / 2; x += spacing) {
            for (let y = -region.height / 2 + spacing/2; y < region.height / 2; y += spacing) {
                this.drawArrow(new Vector2D(x, y), region.electricField.normalize().multiply(15));
            }
        }
    }

    // Draw Magnetic Field symbols
    if (Math.abs(region.magneticField) > 0.1) {
        this.ctx.strokeStyle = '#EC4899'; // pink-500
        this.ctx.lineWidth = 1.5 / this.zoom;
        const r = 5 / this.zoom;
        for (let x = -region.width / 2 + spacing/2; x < region.width / 2; x += spacing) {
            for (let y = -region.height / 2 + spacing/2; y < region.height / 2; y += spacing) {
                this.ctx.beginPath();
                this.ctx.arc(x, y, r, 0, Math.PI * 2);
                this.ctx.stroke();
                if (region.magneticField > 0) { // Out of page
                    this.ctx.beginPath();
                    this.ctx.arc(x, y, 1 / this.zoom, 0, Math.PI * 2);
                    this.ctx.fillStyle = '#EC4899';
                    this.ctx.fill();
                } else { // Into page
                    this.ctx.moveTo(x - r * 0.7, y - r * 0.7);
                    this.ctx.lineTo(x + r * 0.7, y + r * 0.7);
                    this.ctx.moveTo(x + r * 0.7, y - r * 0.7);
                    this.ctx.lineTo(x - r * 0.7, y + r * 0.7);
                    this.ctx.stroke();
                }
            }
        }
    }
  }

  private drawArrow(from: Vector2D, vec: Vector2D): void {
      const to = from.add(vec);
      this.ctx.beginPath();
      this.ctx.moveTo(from.x, from.y);
      this.ctx.lineTo(to.x, to.y);
      this.ctx.stroke();

      const angle = Math.atan2(vec.y, vec.x);
      const headLength = 8 / this.zoom;
      this.ctx.beginPath();
      this.ctx.moveTo(to.x, to.y);
      this.ctx.lineTo(to.x - headLength * Math.cos(angle - Math.PI / 6), to.y - headLength * Math.sin(angle - Math.PI / 6));
      this.ctx.moveTo(to.x, to.y);
      this.ctx.lineTo(to.x - headLength * Math.cos(angle + Math.PI / 6), to.y - headLength * Math.sin(angle + Math.PI / 6));
      this.ctx.stroke();
  }

  private drawGhostObject(): void {
    const type = this.placingObjectType()?.();
    if (!type || this.placementState.isPlacing) return;
      this.ctx.save();
      this.ctx.globalAlpha = 0.5;
      this.ctx.setLineDash([5 / this.zoom, 5 / this.zoom]);
      switch (type) {
          case 'ball': this.drawBall({ position: this.mousePosWorld, radius: 20, color: 'white' } as Ball); break;
          case 'block': this.drawBlock({ position: this.mousePosWorld, width: 50, height: 50, angle: 0, color: 'white' } as Block); break;
          case 'conveyor': this.drawConveyor({ start: this.mousePosWorld.subtract(new Vector2D(150, 0)), end: this.mousePosWorld.add(new Vector2D(150, 0)) } as Conveyor); break;
          case 'pin': this.drawPin({ position: this.mousePosWorld } as Pin); break;
          case 'field': this.drawFieldRegion({ position: this.mousePosWorld, width: 200, height: 200, angle: 0, electricField: new Vector2D(), magneticField: 0, color: 'white' } as FieldRegion); break;
      }
      this.ctx.restore();
  }

  private drawPlacementPreview(): void {
    if (!this.placementState.isPlacing) return;
    this.ctx.save();
    this.ctx.globalAlpha = 0.7;
    this.ctx.setLineDash([8 / this.zoom, 8 / this.zoom]);
    const { type, step, startPoint, object1Id, centerPoint, radius, startAngle } = this.placementState;
    if (type === 'plane' && step === 'end' && startPoint) {
        this.ctx.beginPath(); this.ctx.moveTo(startPoint.x, startPoint.y);
        this.ctx.lineTo(this.mousePosWorld.x, this.mousePosWorld.y);
        this.ctx.strokeStyle = '#a0aec0'; this.ctx.lineWidth = 5 / this.zoom; this.ctx.stroke();
    } else if ((type === 'spring' || type === 'rod') && step === 'end' && object1Id) {
        const obj1 = this.objectsMap().get(object1Id);
        if (obj1 && 'position' in obj1) {
            let endPoint = this.mousePosWorld;
            const connectableHovered = this.hoveredObject && 
                                      (this.hoveredObject.type === 'ball' || this.hoveredObject.type === 'block' || this.hoveredObject.type === 'pin') &&
                                      this.hoveredObject.id !== object1Id;
            if (connectableHovered) {
                endPoint = (this.hoveredObject as DynamicObject | Pin).position;
            }

            this.ctx.beginPath(); this.ctx.moveTo(obj1.position.x, obj1.position.y);
            this.ctx.lineTo(endPoint.x, endPoint.y);
            this.ctx.strokeStyle = type === 'spring' ? '#f6e05e' : '#e2e8f0';
            this.ctx.lineWidth = (type === 'spring' ? 2 : 6) / this.zoom;
            this.ctx.stroke();
        }
    } else if (type === 'arc') {
        if (step === 'radius_start' && centerPoint) {
            const r = this.mousePosWorld.subtract(centerPoint).magnitude();
            this.ctx.beginPath(); this.ctx.arc(centerPoint.x, centerPoint.y, r, 0, Math.PI * 2);
            this.ctx.strokeStyle = '#a0aec0'; this.ctx.lineWidth = 5 / this.zoom; this.ctx.stroke();
        } else if (step === 'end_angle' && centerPoint && radius != null && startAngle != null) {
            const endAngle = Math.atan2(this.mousePosWorld.y - centerPoint.y, this.mousePosWorld.x - centerPoint.x);
            this.ctx.beginPath();
            this.ctx.arc(centerPoint.x, centerPoint.y, radius, startAngle, endAngle);
            this.ctx.strokeStyle = '#a0aec0'; this.ctx.lineWidth = 5 / this.zoom; this.ctx.stroke();
        }
    }
    this.ctx.restore();
  }

  private drawSnapIndicator(): void {
      if (!this.snappedObject) return;
      this.ctx.save();
      const pos = (this.snappedObject as any).position;
      if (pos) {
          this.ctx.beginPath();
          this.ctx.arc(pos.x, pos.y, SNAP_DISTANCE / this.zoom, 0, Math.PI * 2);
          this.ctx.strokeStyle = 'rgba(0, 255, 255, 0.7)';
          this.ctx.lineWidth = 2 / this.zoom;
          this.ctx.stroke();
      }
      this.ctx.restore();
  }

  // --- Event Handlers ---
  private onMouseDown = (event: MouseEvent): void => {
    const worldPos = this.getMouseWorldPos(event);
    if (event.button === 1) { // Middle mouse for panning
        this.isPanning = true;
        this.lastPanPosition = new Vector2D(event.clientX, event.clientY);
        this.updateCursor();
        return;
    }
    
    if (this.handlePlacementClick(worldPos)) return;
    if (this.placingObjectType()?.()) {
        this.placeObject.emit(worldPos);
        return;
    }

    const clickedObject = this.getObjectAtPosition(worldPos);
    this.selectObject.emit(clickedObject?.id ?? null);

    if (clickedObject?.type === 'ball' || clickedObject?.type === 'block' || clickedObject?.type === 'field') {
      this.isDragging = true;
      this.dragTarget = clickedObject;
      this.dragOffset = worldPos.subtract(clickedObject.position);
      this.updateCursor();
    }
  }

  private onMouseMove = (event: MouseEvent): void => {
    if (this.isPanning) {
        const currentPanPos = new Vector2D(event.clientX, event.clientY);
        const delta = currentPanPos.subtract(this.lastPanPosition);
        this.panOffset = this.panOffset.add(delta);
        this.lastPanPosition = currentPanPos;
        this.draw();
        return;
    }

    this.mousePosWorld = this.getMouseWorldPos(event);
    this.hoveredObject = this.getObjectAtPosition(this.mousePosWorld);
    
    if (this.isDragging && this.dragTarget) {
      let targetPos = this.mousePosWorld.subtract(this.dragOffset);
      this.snappedObject = null;
      let isStrongSnapped = false;

      if (this.dragTarget.type === 'ball' || this.dragTarget.type === 'block') {
          // New: Plane surface snapping for balls
          if (this.dragTarget.type === 'ball') {
              for (const obj of this.objects()()) {
                  if (obj.type === 'plane' || obj.type === 'conveyor') {
                      if (this.distToSegment(targetPos, obj.start, obj.end) < SNAP_DISTANCE / this.zoom) {
                          const lineVec = obj.end.subtract(obj.start);
                          const normal = new Vector2D(-lineVec.y, lineVec.x).normalize();
                          const vecToStart = targetPos.subtract(obj.start);
                          const projDist = vecToStart.dot(lineVec.normalize());
                          const projectedPoint = obj.start.add(lineVec.normalize().multiply(projDist));

                          targetPos = projectedPoint.add(normal.multiply(this.dragTarget.radius));
                          this.snappedObject = obj;
                          isStrongSnapped = true;
                          break;
                      }
                  }
              }
          }

          // Original center-to-center snapping
          if (!isStrongSnapped) {
              for (const obj of this.objects()()) {
                  if (obj.id === this.dragTarget.id || !('position' in obj)) continue;
                  if (targetPos.subtract(obj.position).magnitude() < SNAP_DISTANCE / this.zoom) {
                      targetPos = obj.position.clone();
                      this.snappedObject = obj;
                      isStrongSnapped = true;
                      break;
                  }
              }
          }
      }
      
      // Weak axis snapping
      if (!isStrongSnapped) {
          const weakSnapDist = WEAK_SNAP_DISTANCE / this.zoom;
          if (Math.abs(targetPos.x) < weakSnapDist) {
              targetPos.x = 0;
          }
          if (Math.abs(targetPos.y) < weakSnapDist) {
              targetPos.y = 0;
          }
      }

      this.dragTarget.position = targetPos;
      if (this.dragTarget.type === 'ball' || this.dragTarget.type === 'block') {
        this.dragTarget.velocity = new Vector2D(0, 0);
        if(this.dragTarget.type === 'block') this.dragTarget.angularVelocity = 0;
      }
    }

    this.updateCursor();
    if (!this.settings().isRunning) { this.draw(); }
  }

  private onMouseUp = (event: MouseEvent): void => {
    this.isDragging = false; this.dragTarget = null;
    this.isPanning = false; this.snappedObject = null;
    this.updateCursor();
    if (!this.settings().isRunning) { this.draw(); }
  }

  private onMouseLeave = (event: MouseEvent): void => { this.onMouseUp(event); }
  
  private onWheel = (event: WheelEvent): void => {
    event.preventDefault();
    const zoomAmount = event.deltaY > 0 ? 0.9 : 1.1;
    this.applyZoom(zoomAmount, this.getScreenPos(event));
  }

  // --- Helpers ---
  private resizeCanvas = (): void => {
    const canvas = this.canvasRef.nativeElement;
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
    this.draw();
  }

  private applyZoom(amount: number, screenPos?: Vector2D): void {
      screenPos = screenPos || new Vector2D(this.canvasRef.nativeElement.width / 2, this.canvasRef.nativeElement.height / 2);
      const worldPosBefore = this.getWorldPos(screenPos);
      this.zoom = Math.max(0.1, Math.min(10, this.zoom * amount));
      const worldPosAfter = this.getWorldPos(screenPos);
      this.panOffset = this.panOffset.add(worldPosAfter.subtract(worldPosBefore).multiply(this.zoom));
      this.draw();
  }

  private updateCursor(): void {
    const canvas = this.canvasRef.nativeElement;
    if (this.isPanning) canvas.style.cursor = 'grabbing';
    else if (this.placingObjectType()?.()) canvas.style.cursor = 'crosshair';
    else if (this.isDragging) canvas.style.cursor = 'grabbing';
    else if (this.hoveredObject) canvas.style.cursor = 'pointer';
    else canvas.style.cursor = 'grab';
  }

  private getScreenPos = (event: MouseEvent): Vector2D => {
      const rect = this.canvasRef.nativeElement.getBoundingClientRect();
      return new Vector2D(event.clientX - rect.left, event.clientY - rect.top);
  }
  
  private getMouseWorldPos = (event: MouseEvent): Vector2D => this.getWorldPos(this.getScreenPos(event));
  private getWorldPos = (screenPos: Vector2D): Vector2D => screenPos.subtract(this.panOffset).multiply(1 / this.zoom);

  private handlePlacementClick(worldPos: Vector2D): boolean {
    if (!this.placementState.isPlacing) return false;
    const { type, step, startPoint, object1Id, centerPoint, radius, startAngle } = this.placementState;

    switch (type) {
      case 'plane':
        if (step === 'start') {
          this.placementState.startPoint = worldPos;
          this.placementState.step = 'end';
        } else {
          this.placeObject.emit({ start: startPoint, end: worldPos });
        }
        return true;
      case 'spring':
      case 'rod': {
        const clickableObj = this.getObjectAtPosition(worldPos, true);

        if (step === 'start') {
          if (clickableObj) {
            this.placementState.object1Id = clickableObj.id;
            this.placementState.step = 'end';
          }
        } else if (object1Id) {
          if (clickableObj && clickableObj.id !== object1Id) {
            this.placeObject.emit({ object1Id, object2Id: clickableObj.id });
          } else {
            // Clicked on empty space or same object, cancel placement
            this.placementState = { isPlacing: false, type: null, step: 'start' };
          }
        }
        return true;
      }
      case 'arc':
        if (step === 'center') {
          this.placementState.centerPoint = worldPos;
          this.placementState.step = 'radius_start';
        } else if (step === 'radius_start' && centerPoint) {
          const vec = worldPos.subtract(centerPoint);
          this.placementState.radius = vec.magnitude();
          this.placementState.startAngle = Math.atan2(vec.y, vec.x);
          this.placementState.step = 'end_angle';
        } else if (step === 'end_angle' && centerPoint && radius != null && startAngle != null) {
          const endAngle = Math.atan2(worldPos.y - centerPoint.y, worldPos.x - centerPoint.x);
          this.placeObject.emit({ center: centerPoint, radius, startAngle, endAngle });
        }
        return true;
      default:
        return false;
    }
  }
  
  private getObjectAtPosition(pos: Vector2D, connectableOnly: boolean = false): PhysicsObject | null {
    const tolerance = CLICK_TOLERANCE / this.zoom;
    const objects = [...this.objects()()].reverse();
    for (const obj of objects) {
        if (connectableOnly && !(obj.type === 'ball' || obj.type === 'block' || obj.type === 'pin')) continue;
        switch (obj.type) {
            case 'ball': if (pos.subtract(obj.position).magnitude() < obj.radius) return obj; break;
            case 'block':
            case 'field':
                const localPos = pos.subtract(obj.position);
                const rotatedPos = new Vector2D(
                    localPos.x * Math.cos(-obj.angle) - localPos.y * Math.sin(-obj.angle),
                    localPos.x * Math.sin(-obj.angle) + localPos.y * Math.cos(-obj.angle)
                );
                if (Math.abs(rotatedPos.x) < obj.width / 2 && Math.abs(rotatedPos.y) < obj.height / 2) return obj;
                break;
            case 'pin': if (pos.subtract(obj.position).magnitude() < PIN_CLICK_RADIUS / this.zoom) return obj; break;
            case 'plane':
            case 'conveyor': if (this.distToSegment(pos, obj.start, obj.end) < tolerance) return obj; break;
            case 'rod':
            case 'spring':
                const points = this.getConnectorPoints(obj);
                if (points && this.distToSegment(pos, points.p1, points.p2) < tolerance) return obj;
                break;
            case 'arc': {
                const toCenter = pos.subtract(obj.center);
                const dist = toCenter.magnitude();

                if (Math.abs(dist - obj.radius) < tolerance) {
                    let angle = Math.atan2(toCenter.y, toCenter.x);
                    const start = obj.startAngle;
                    const end = obj.endAngle;

                    const normalize = (a: number) => (a % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
                    const normAngle = normalize(angle);
                    const normStart = normalize(start);
                    const normEnd = normalize(end);
                    
                    if (normStart <= normEnd) {
                        if (normAngle >= normStart && normAngle <= normEnd) return obj;
                    } else { // Arc crosses 0 radian line
                        if (normAngle >= normStart || normAngle <= normEnd) return obj;
                    }
                }
                break;
            }
        }
    }
    return null;
  }
  
  private getConnectorPoints = (obj: Spring | Rod): {p1: Vector2D, p2: Vector2D} | null => {
      const allObjectsMap = this.objectsMap();
      const obj1 = allObjectsMap.get(obj.object1Id);
      const obj2 = allObjectsMap.get(obj.object2Id);
      if (obj1 && obj2 && 'position' in obj1 && 'position' in obj2) {
          return { p1: obj1.position, p2: obj2.position };
      }
      return null;
  }
  
  private distToSegment(p: Vector2D, v: Vector2D, w: Vector2D): number {
    const l2 = v.subtract(w).magnitude() ** 2;
    if (l2 == 0) return p.subtract(v).magnitude();
    let t = Math.max(0, Math.min(1, p.subtract(v).dot(w.subtract(v)) / l2));
    const projection = v.add(w.subtract(v).multiply(t));
    return p.subtract(projection).magnitude();
  }
  
  private getBlockVertices(block: Block): Vector2D[] {
      const w = block.width / 2; const h = block.height / 2;
      const cos = Math.cos(block.angle); const sin = Math.sin(block.angle);
      const vertices: Vector2D[] = [new Vector2D(-w, -h), new Vector2D(w, -h), new Vector2D(w, h), new Vector2D(-w, h)];
      return vertices.map(v => new Vector2D(
        v.x * cos - v.y * sin + block.position.x,
        v.x * sin + v.y * cos + block.position.y
      ));
  }

  // SAT Helper Methods
  private getBlockAxes(vertices: Vector2D[]): Vector2D[] {
    const axes: Vector2D[] = [];
    for (let i = 0; i < vertices.length; i++) {
        const p1 = vertices[i];
        const p2 = vertices[i + 1] || vertices[0];
        const edge = p2.subtract(p1);
        axes.push(new Vector2D(-edge.y, edge.x).normalize());
    }
    return axes;
  }

  private projectVertices(vertices: Vector2D[], axis: Vector2D): { min: number, max: number } {
    let min = Infinity, max = -Infinity;
    for (const vertex of vertices) {
        const proj = vertex.dot(axis);
        if (proj < min) min = proj;
        if (proj > max) max = proj;
    }
    return { min, max };
  }
    private projectBall(ball: Ball, axis: Vector2D): {min: number, max: number} {
        const centerProj = ball.position.dot(axis);
        return { min: centerProj - ball.radius, max: centerProj + ball.radius };
    }

    private findClosestVertexToPoint(vertices: Vector2D[], point: Vector2D): Vector2D {
        let closestVertex = vertices[0];
        let minDistSq = Infinity;
        for(const v of vertices) {
            const distSq = v.subtract(point).magnitude() ** 2;
            if(distSq < minDistSq) {
                minDistSq = distSq;
                closestVertex = v;
            }
        }
        return closestVertex;
    }

    private findContactPoint(objA: Ball | Block, objB: Ball | Block): Vector2D {
        // Simplified contact point finding
        if (objA.type === 'block' && objB.type === 'block') {
            const verticesA = this.getBlockVertices(objA);
            const verticesB = this.getBlockVertices(objB);
            let minDistSq = Infinity;
            let contactPoint = new Vector2D();
            for(const v of verticesA) {
                const closestOnB = this.findClosestVertexToPoint(verticesB, v);
                const distSq = v.subtract(closestOnB).magnitude() ** 2;
                if(distSq < minDistSq) {
                    minDistSq = distSq;
                    contactPoint = v.add(closestOnB).multiply(0.5);
                }
            }
             for(const v of verticesB) {
                const closestOnA = this.findClosestVertexToPoint(verticesA, v);
                const distSq = v.subtract(closestOnA).magnitude() ** 2;
                if(distSq < minDistSq) {
                    minDistSq = distSq;
                    contactPoint = v.add(closestOnA).multiply(0.5);
                }
            }
            return contactPoint;
        }
        return objA.position.add(objB.position).multiply(0.5);
    }
}
