import { Vector2D } from './vector.model';

export type ObjectType = 'ball' | 'block' | 'plane' | 'spring' | 'conveyor' | 'arc' | 'rod' | 'pin' | 'field';

export interface BaseObject {
  id: number;
  type: ObjectType;
  isSelected: boolean;
}

export interface DynamicObject extends BaseObject {
  position: Vector2D;
  velocity: Vector2D;
  acceleration: Vector2D;
  force: Vector2D;
  mass: number;
  color: string;
  restitution: number;
  friction: number;
  charge: number;
}

export interface StaticObject extends BaseObject {
  friction: number; // Coefficient of friction
  restitution: number;
}

export interface Ball extends DynamicObject {
  type: 'ball';
  radius: number;
}

export interface Block extends DynamicObject {
  type: 'block';
  width: number;
  height: number;
  angle: number; // in radians
  angularVelocity: number;
  momentOfInertia: number;
  torque: number;
}

export interface Plane extends StaticObject {
  type: 'plane';
  start: Vector2D;
  end: Vector2D;
}

export interface Spring extends BaseObject {
  type: 'spring';
  object1Id: number;
  object2Id: number;
  stiffness: number; // k
  restLength: number;
}

export interface Rod extends BaseObject {
  type: 'rod';
  object1Id: number;
  object2Id: number;
  length: number;
}

export interface Pin extends BaseObject {
  type: 'pin';
  position: Vector2D;
}

export interface Conveyor extends StaticObject {
  type: 'conveyor';
  start: Vector2D;
  end: Vector2D;
  speed: number;
}

export interface Arc extends StaticObject {
  type: 'arc';
  center: Vector2D;
  radius: number;
  startAngle: number; // radians
  endAngle: number; // radians
}

export interface FieldRegion extends BaseObject {
  type: 'field';
  position: Vector2D;
  width: number;
  height: number;
  angle: number;
  electricField: Vector2D;
  magneticField: number;
  color: string;
}


export type PhysicsObject = Ball | Block | Plane | Spring | Conveyor | Arc | Rod | Pin | FieldRegion;

export interface GraphDataPoint {
  t: number;
  x: number;
  y: number;
  vx: number;
  vy: number;
}