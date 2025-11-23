export class Vector2D {
  constructor(public x: number = 0, public y: number = 0) {}

  add(v: Vector2D): Vector2D {
    return new Vector2D(this.x + v.x, this.y + v.y);
  }

  subtract(v: Vector2D): Vector2D {
    return new Vector2D(this.x - v.x, this.y - v.y);
  }

  multiply(scalar: number): Vector2D {
    return new Vector2D(this.x * scalar, this.y * scalar);
  }

  magnitude(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }

  normalize(): Vector2D {
    const mag = this.magnitude();
    if (mag === 0) {
      return new Vector2D(0, 0);
    }
    return new Vector2D(this.x / mag, this.y / mag);
  }

  dot(v: Vector2D): number {
    return this.x * v.x + this.y * v.y;
  }

  clone(): Vector2D {
    return new Vector2D(this.x, this.y);
  }

  toJSON() {
    return { x: this.x, y: this.y };
  }
}
