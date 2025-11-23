import { Component, ChangeDetectionStrategy, signal, WritableSignal, ViewChild, computed } from '@angular/core';
import { ObjectPaletteComponent } from './components/object-palette/object-palette.component';
import { PropertiesEditorComponent } from './components/properties-editor/properties-editor.component';
import { SimulationCanvasComponent } from './components/simulation-canvas/simulation-canvas.component';
import { GraphViewComponent } from './components/graph-view/graph-view.component';
import { PhysicsObject, ObjectType, Arc, DynamicObject, Pin, Rod, Block, GraphDataPoint, FieldRegion, FieldShape, PolygonalFieldRegion, Plane } from './models/physics-objects.model';
import { Vector2D } from './models/vector.model';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  changeDetection: ChangeDetectionStrategy.OnPush,
  imports: [
    ObjectPaletteComponent,
    PropertiesEditorComponent,
    SimulationCanvasComponent,
    GraphViewComponent
  ]
})
export class AppComponent {
  private objectIdCounter = 0;

  physicsObjects: WritableSignal<PhysicsObject[]> = signal<PhysicsObject[]>([
    { id: this.getNextId(), type: 'plane', start: new Vector2D(50, 500), end: new Vector2D(750, 500), friction: 0.2, restitution: 0.5, isSelected: false }
  ]);
  selectedObject: WritableSignal<PhysicsObject | null> = signal(null);
  placingObjectType: WritableSignal<ObjectType | null> = signal(null);
  placingFieldShape: WritableSignal<FieldShape | null> = signal(null);
  
  simulationSettings = signal({
    gravity: 9.8,
    isRunning: false,
    collisionsEnabled: true,
  });
  
  graphData: WritableSignal<GraphDataPoint[]> = signal([]);

  // Pinned objects feature
  pinnedObjectIds = signal<number[]>([]);
  pinnedObjects = computed(() => {
    const objectsMap = new Map(this.physicsObjects().map(o => [o.id, o]));
    return this.pinnedObjectIds().map(id => objectsMap.get(id)).filter((o): o is PhysicsObject => !!o);
  });
  
  // Function plot feature
  showFunctionDialog = signal(false);
  functionPlotOptions = {
    functionString: '100 * Math.sin(x / 50)',
    xMin: 0,
    xMax: 400,
    segments: 40,
    type: 'track' as 'track' | 'field',
    yOffset: 300
  };

  @ViewChild(SimulationCanvasComponent) simulationCanvas!: SimulationCanvasComponent;

  private getNextId(): number {
    return this.objectIdCounter++;
  }

  startPlacingObject(payload: { type: ObjectType, shape?: FieldShape }): void {
    if (payload.type === 'functionPlot') {
        this.showFunctionDialog.set(true);
        return;
    }
    this.placingObjectType.set(payload.type);
    if (payload.type === 'field' && payload.shape) {
        this.placingFieldShape.set(payload.shape);
    } else {
        this.placingFieldShape.set(null);
    }
    this.selectObject(null);
  }

  placeObject(payload: any): void {
    const type = this.placingObjectType();
    if (!type) return;

    let newObject: PhysicsObject | null = null;
    const id = this.getNextId();
    const randomColor = `hsl(${Math.random() * 360}, 90%, 70%)`;

    switch (type) {
      case 'ball':
        newObject = {
          id, type, position: payload, velocity: new Vector2D(0, 0),
          acceleration: new Vector2D(0, 0), force: new Vector2D(0, 0), mass: 1,
          radius: 20, isSelected: false, color: randomColor, restitution: 0.8, friction: 0.3, charge: 0,
        };
        break;
      case 'block':
        const mass = 5;
        const width = 50;
        const height = 50;
        newObject = {
          id, type, position: payload, velocity: new Vector2D(0, 0),
          acceleration: new Vector2D(0, 0), force: new Vector2D(0, 0), mass: mass,
          width: width, height: height, angle: 0, isSelected: false, color: randomColor, restitution: 0.5, friction: 0.4,
          angularVelocity: 0,
          momentOfInertia: (1/12) * mass * (width*width + height*height),
          torque: 0,
          charge: 0,
        } as Block;
        break;
      case 'plane':
        newObject = {
          id, type, start: payload.start, end: payload.end,
          friction: 0.2, isSelected: false, restitution: 0.5
        };
        break;
      case 'conveyor':
        newObject = {
          id, type, start: payload.start, end: payload.end,
          friction: 0.1, speed: 50, isSelected: false, restitution: 0.5
        };
        break;
      case 'spring':
      case 'rod':
        const obj1 = this.physicsObjects().find(o => o.id === payload.object1Id);
        const obj2 = this.physicsObjects().find(o => o.id === payload.object2Id);
        
        if (!obj1 || !obj2) return;

        const pos1 = (obj1 as any).position as Vector2D | undefined;
        const pos2 = (obj2 as any).position as Vector2D | undefined;

        if (!pos1 || !pos2) return;
        const dist = pos1.subtract(pos2).magnitude();

        if (type === 'spring') {
          newObject = {
            id, type,
            object1Id: payload.object1Id,
            object2Id: payload.object2Id,
            stiffness: 100,
            restLength: dist,
            isSelected: false
          };
        } else { // rod
          newObject = {
            id, type,
            object1Id: payload.object1Id,
            object2Id: payload.object2Id,
            length: dist,
            isSelected: false
          } as Rod;
        }
        break;
      case 'arc':
        newObject = {
          id, type, 
          center: payload.center, 
          radius: payload.radius,
          startAngle: payload.startAngle, 
          endAngle: payload.endAngle,
          friction: 0.2, isSelected: false, restitution: 0.5
        } as Arc;
        break;
      case 'pin':
        newObject = {
          id, type, position: payload, isSelected: false
        } as Pin;
        break;
      case 'field':
        const shape = this.placingFieldShape();
        if (!shape) return;

        switch(shape) {
            case 'rectangle': {
                const start = payload.start as Vector2D;
                const end = payload.end as Vector2D;
                const rectWidth = Math.abs(end.x - start.x);
                const rectHeight = Math.abs(end.y - start.y);
                const position = new Vector2D((start.x + end.x) / 2, (start.y + end.y) / 2);
                newObject = {
                    id, type, shape, position: position,
                    width: rectWidth, height: rectHeight, angle: 0,
                    electricField: new Vector2D(50, 0), magneticField: 0,
                    isSelected: false, color: '#4A5568'
                };
                break;
            }
            case 'circle': {
                newObject = {
                    id, type, shape, position: payload.center, radius: payload.radius,
                    electricField: new Vector2D(50, 0), magneticField: 0,
                    isSelected: false, color: '#4A5568'
                };
                break;
            }
            case 'polygon': {
                const vertices = payload.vertices as Vector2D[];
                if (vertices.length < 3) return;
                // Calculate centroid
                let signedArea = 0;
                let cx = 0, cy = 0;
                for (let i = 0; i < vertices.length; i++) {
                    const v0 = vertices[i];
                    const v1 = vertices[(i + 1) % vertices.length];
                    const cross = (v0.x * v1.y - v1.x * v0.y);
                    signedArea += cross;
                    cx += (v0.x + v1.x) * cross;
                    cy += (v0.y + v1.y) * cross;
                }
                signedArea /= 2;
                const centroid = signedArea === 0 ? payload.vertices[0] : new Vector2D(cx / (6 * signedArea), cy / (6 * signedArea));
                
                newObject = {
                    id, type, shape, vertices: vertices, position: centroid,
                    electricField: new Vector2D(50, 0), magneticField: 0,
                    isSelected: false, color: '#4A5568'
                } as PolygonalFieldRegion;
                break;
            }
        }
        break;
    }

    if (newObject) {
      this.physicsObjects.update(objects => [...objects, newObject!]);
      this.selectObject(newObject.id);
    }
    this.placingObjectType.set(null);
    this.placingFieldShape.set(null);
  }
  
  selectObject(id: number | null): void {
    let newSelectedObject: PhysicsObject | null = null;
    this.physicsObjects.update(objects => {
      return objects.map(obj => {
        if (obj.id === id) {
          obj.isSelected = true;
          newSelectedObject = obj;
        } else {
          obj.isSelected = false;
        }
        return obj;
      });
    });
    this.selectedObject.set(newSelectedObject);
    if (this.simulationSettings().isRunning && newSelectedObject) {
        this.graphData.set([]);
        this.simulationCanvas?.resetTime();
    }
  }

  updateObject(updatedObject: PhysicsObject): void {
    this.physicsObjects.update(objects => {
      const index = objects.findIndex(obj => obj.id === updatedObject.id);
      if (index > -1) {
        const newObjects = [...objects];
        newObjects[index] = updatedObject;
        // Also update the selected object signal if it's the same object
        if (this.selectedObject()?.id === updatedObject.id) {
            this.selectedObject.set(updatedObject);
        }
        return newObjects;
      }
      return objects;
    });
  }

  deleteSelectedObject(): void {
    const selected = this.selectedObject();
    if (selected) {
      this.physicsObjects.update(objects => objects.filter(obj => obj.id !== selected.id));
      this.selectedObject.set(null);
    }
  }
  
  updateGravity(value: string): void {
    this.simulationSettings.update(s => ({ ...s, gravity: parseFloat(value) || 0 }));
  }

  togglePlay(): void {
    if (!this.simulationSettings().isRunning && this.selectedObject()) { // About to start
      this.graphData.set([]);
      this.simulationCanvas?.resetTime();
    }
    this.simulationSettings.update(s => ({ ...s, isRunning: !s.isRunning }));
  }
  
  toggleCollisions(): void {
    this.simulationSettings.update(s => ({ ...s, collisionsEnabled: !s.collisionsEnabled }));
  }

  clearSimulation(): void {
    this.physicsObjects.set([]);
    this.selectedObject.set(null);
    this.objectIdCounter = 0;
    this.graphData.set([]);
    this.pinnedObjectIds.set([]);
  }

  addDataPoint(point: GraphDataPoint): void {
    this.graphData.update(data => [...data, point]);
  }

  makeAllSmooth(): void {
    this.physicsObjects.update(objects => 
        objects.map(obj => {
            if (obj.type === 'plane' || obj.type === 'arc' || obj.type === 'conveyor') {
                return { ...obj, friction: 0 };
            }
            return obj;
        })
    );
    // If selected object was smoothed, update the signal to refresh properties editor
    const selected = this.selectedObject();
    if (selected && (selected.type === 'plane' || selected.type === 'arc' || selected.type === 'conveyor')) {
        this.selectedObject.update(obj => obj ? ({ ...obj, friction: 0 }) : null);
    }
  }

  togglePin(id: number): void {
    this.pinnedObjectIds.update(ids => {
        const index = ids.indexOf(id);
        if (index > -1) {
            return ids.filter(pinnedId => pinnedId !== id);
        } else {
            return [...ids, id];
        }
    });
  }
  
  createFunctionObject(): void {
    try {
        const fn = new Function('x', `return ${this.functionPlotOptions.functionString}`);
        const points: Vector2D[] = [];
        const { xMin, xMax, segments, yOffset } = this.functionPlotOptions;
        const step = (xMax - xMin) / segments;

        for(let i = 0; i <= segments; i++) {
            const x = xMin + i * step;
            const y = fn(x) + yOffset;
            if (isNaN(y) || !isFinite(y)) throw new Error(`Invalid function output at x=${x}`);
            points.push(new Vector2D(x, y));
        }
        
        const newObjects: PhysicsObject[] = [];
        if (this.functionPlotOptions.type === 'track') {
            for(let i = 0; i < points.length - 1; i++) {
                newObjects.push({
                    id: this.getNextId(),
                    type: 'plane',
                    start: points[i],
                    end: points[i+1],
                    friction: 0.2,
                    restitution: 0.5,
                    isSelected: false,
                } as Plane);
            }
        } else { // field
            const groundLevel = Math.max(...points.map(p => p.y)) + 50;
            const vertices = [
                ...points,
                new Vector2D(xMax, groundLevel),
                new Vector2D(xMin, groundLevel)
            ];
            
            this.placeObject({ vertices }); // Use the existing polygon field placement logic
        }

        if (newObjects.length > 0) {
            this.physicsObjects.update(current => [...current, ...newObjects]);
        }

    } catch(e) {
        console.error("Function plot error:", e);
        alert(`Invalid function or range: ${e instanceof Error ? e.message : String(e)}`);
    }
    this.showFunctionDialog.set(false);
  }

  // Canvas interaction methods
  zoomIn(): void { this.simulationCanvas?.zoomIn(); }
  zoomOut(): void { this.simulationCanvas?.zoomOut(); }
  resetView(): void { this.simulationCanvas?.resetView(); }
  exportImage(): void { this.simulationCanvas?.exportAsImage(); }

  saveSimulation(): void {
    try {
      const dataStr = JSON.stringify(this.physicsObjects(), (key, value) => {
        // Custom replacer to handle Vector2D
        if (value instanceof Vector2D) {
          return { x: value.x, y: value.y, __type: 'Vector2D' };
        }
        return value;
      }, 2);
      const blob = new Blob([dataStr], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `physics-sim-${new Date().toISOString().slice(0,10)}.json`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    } catch (error) {
      console.error("Failed to save simulation:", error);
      alert("Error: Could not save simulation data.");
    }
  }

  loadSimulation(event: Event): void {
    const input = event.target as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      const text = e.target?.result as string;
      try {
        const objects = JSON.parse(text, (key, value) => {
          if (value && typeof value === 'object' && value.__type === 'Vector2D') {
            return new Vector2D(value.x, value.y);
          }
          if (value && typeof value === 'object' && 'x' in value && 'y' in value && Object.keys(value).length === 2) {
             return new Vector2D(value.x, value.y);
          }
          return value;
        });

        if (!Array.isArray(objects)) throw new Error("Invalid file format");

        // Migration for old field objects
        objects.forEach((obj: any) => {
          if (obj.type === 'field' && !obj.shape) {
            obj.shape = 'rectangle';
          }
        });

        this.physicsObjects.set(objects);
        this.selectedObject.set(null);
        this.objectIdCounter = Math.max(0, ...objects.map((o: PhysicsObject) => o.id)) + 1;

      } catch (error) {
        console.error("Error loading or parsing simulation file:", error);
        alert("Error: Could not load the simulation file. It may be invalid or corrupted.");
      }
    };
    reader.onerror = () => {
        alert("Error reading file.");
    };
    reader.readAsText(file);
    input.value = ''; // Reset for same-file uploads
  }
}
