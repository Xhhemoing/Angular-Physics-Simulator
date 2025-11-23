import { Component, ChangeDetectionStrategy, input, output, computed } from '@angular/core';
import { PhysicsObject, Ball, Block, Plane, Conveyor, Spring, Rod, Pin, FieldRegion, PolygonalFieldRegion } from '../../models/physics-objects.model';
import { Vector2D } from '../../models/vector.model';

@Component({
  selector: 'app-properties-editor',
  templateUrl: './properties-editor.component.html',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class PropertiesEditorComponent {
  selectedObject = input<PhysicsObject | null>();
  pinnedIds = input.required<number[]>();

  objectChange = output<PhysicsObject>();
  deleteObject = output<void>();
  pinObject = output<number>();

  isPinned = computed(() => {
    const obj = this.selectedObject();
    return obj ? this.pinnedIds().includes(obj.id) : false;
  });

  onValueChange(property: string, value: string | number): void {
    const obj = this.selectedObject();
    if (!obj) return;

    // Create a deep enough copy to modify nested properties
    const newObj = JSON.parse(JSON.stringify(obj));
    const numValue = typeof value === 'string' ? (parseFloat(value) || 0) : value;

    const keys = property.split('.');
    let current = newObj;
    for (let i = 0; i < keys.length - 1; i++) {
      current = current[keys[i]];
    }
    current[keys[keys.length - 1]] = numValue;
    
    // Re-instantiate Vector2D objects after JSON stringify/parse
    for (const key of ['position', 'velocity', 'center', 'start', 'end', 'electricField']) {
        if (key in newObj && newObj[key] && typeof newObj[key] === 'object' && 'x' in newObj[key]) {
            newObj[key] = new Vector2D(newObj[key].x, newObj[key].y);
        }
    }
    if (newObj.type === 'field' && newObj.shape === 'polygon') {
      newObj.vertices = (newObj as PolygonalFieldRegion).vertices.map(v => new Vector2D(v.x, v.y));
    }


    if (newObj.type === 'block' && ['mass', 'width', 'height'].includes(keys[0])) {
      const b = newObj as Block;
      b.momentOfInertia = (1/12) * b.mass * (b.width*b.width + b.height*b.height);
    }

    this.objectChange.emit(newObj as PhysicsObject);
  }
  
  onDelete(): void {
    this.deleteObject.emit();
  }

  onPin(): void {
    const obj = this.selectedObject();
    if (obj) {
      this.pinObject.emit(obj.id);
    }
  }
}
