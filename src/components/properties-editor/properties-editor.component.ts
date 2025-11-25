import { Component, ChangeDetectionStrategy, input, output, computed } from '@angular/core';
import { PhysicsObject, Ball, Block, Plane, Conveyor, Spring, Rod, Pin, FieldRegion, PolygonalFieldRegion, DynamicObject } from '../../models/physics-objects.model';
import { Vector2D } from '../../models/vector.model';

@Component({
  selector: 'app-properties-editor',
  templateUrl: './properties-editor.component.html',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class PropertiesEditorComponent {
  selectedObject = input<PhysicsObject | null>();
  pinnedObject = input<PhysicsObject | null>();
  objectChange = output<PhysicsObject>();
  deleteObject = output<void>();
  togglePin = output<void>();

  isPinned = computed(() => this.selectedObject() && this.selectedObject()?.id === this.pinnedObject()?.id);

  private deepCloneWithVectors(obj: any): any {
    if (obj === null || typeof obj !== 'object') {
        return obj;
    }

    if (obj instanceof Vector2D) {
        return obj.clone();
    }

    if (Array.isArray(obj)) {
        return obj.map(item => this.deepCloneWithVectors(item));
    }

    const newObj: { [key: string]: any } = {};
    for (const key in obj) {
        if (Object.prototype.hasOwnProperty.call(obj, key)) {
            newObj[key] = this.deepCloneWithVectors(obj[key]);
        }
    }
    return newObj;
  }

  onValueChange(property: string, value: string | number): void {
    const obj = this.selectedObject();
    if (!obj) return;

    // Use a robust deep clone that preserves Vector2D instances
    const newObj = this.deepCloneWithVectors(obj);
    const numValue = typeof value === 'string' ? (parseFloat(value) || 0) : value;

    const keys = property.split('.');
    let current: any = newObj;
    for (let i = 0; i < keys.length - 1; i++) {
      current = current[keys[i]];
    }
    current[keys[keys.length - 1]] = numValue;

    if (newObj.type === 'block' && ['mass', 'width', 'height'].includes(keys[0])) {
      const b = newObj as Block;
      b.momentOfInertia = (1/12) * b.mass * (b.width*b.width + b.height*b.height);
    }

    this.objectChange.emit(newObj as PhysicsObject);
  }
  
  onDelete(): void {
    this.deleteObject.emit();
  }

  onTogglePin(): void {
    this.togglePin.emit();
  }

  onToggleStatic(): void {
    const obj = this.selectedObject();
    if (!obj || !(obj.type === 'ball' || obj.type === 'block')) return;

    // FIX: Changed type assertion from DynamicObject to Ball | Block to allow for correct type narrowing.
    const newObj = this.deepCloneWithVectors(obj) as Ball | Block;
    newObj.isStatic = !newObj.isStatic;
    
    if (newObj.isStatic) {
        newObj.velocity = new Vector2D(0, 0);
        if (newObj.type === 'block') {
            newObj.angularVelocity = 0;
        }
    }
    this.objectChange.emit(newObj as PhysicsObject);
  }
}