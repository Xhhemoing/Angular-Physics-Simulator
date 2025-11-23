import { Component, ChangeDetectionStrategy, output } from '@angular/core';
import { ObjectType } from '../../models/physics-objects.model';

interface PaletteItem {
  type: ObjectType;
  name: string;
  icon: string; // SVG path
}

@Component({
  selector: 'app-object-palette',
  templateUrl: './object-palette.component.html',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class ObjectPaletteComponent {
  addObject = output<ObjectType>();

  items: PaletteItem[] = [
    { type: 'ball', name: '小球', icon: 'M10 18a8 8 0 100-16 8 8 0 000 16z' },
    { type: 'block', name: '物块', icon: 'M4 3a1 1 0 00-1 1v12a1 1 0 001 1h12a1 1 0 001-1V4a1 1 0 00-1-1H4z' },
    { type: 'plane', name: '平面', icon: 'M17.5 4.5l-15 4' },
    { type: 'arc', name: '圆弧轨道', icon: 'M4 16 A 8 8 0 0 0 16 16' },
    { type: 'conveyor', name: '传送带', icon: 'M3 8l4 4 4-4m-8 5l4 4 4-4' },
    { type: 'spring', name: '弹簧', icon: 'M3 5h2m2 0h2m2 0h2m2 0h2m-1 0a2 2 0 01-2-2V3a2 2 0 012-2h0a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 00-2 2v2a2 2 0 002 2h2a2 2 0 012 2v2a2 2 0 01-2 2h-1' },
    { type: 'rod', name: '硬杆', icon: 'M4 10a2 2 0 100-0.01M16 10a2 2 0 100-0.01M4 10H16' },
    { type: 'pin', name: '固定铰', icon: 'M10 2 v4 M10 14 v4 M2 10 h4 M14 10 h4 M10 10a4 4 0 100-0.01' },
    { type: 'field', name: '场区域', icon: 'M3 4a1 1 0 011-1h12a1 1 0 011 1v12a1 1 0 01-1 1H4a1 1 0 01-1-1V4zm5 5h4m-2-2l2 2-2 2' }
  ];

  onAddObject(type: ObjectType): void {
    this.addObject.emit(type);
  }
}