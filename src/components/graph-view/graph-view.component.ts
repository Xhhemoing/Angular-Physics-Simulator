import { Component, ChangeDetectionStrategy, input, effect, ViewChild, ElementRef, signal, AfterViewInit } from '@angular/core';
import { GraphDataPoint, PhysicsObject } from '../../models/physics-objects.model';

declare const d3: any;

type GraphTab = 'x-t' | 'v-t' | 'x-y' | 'ke-t' | 'length-t' | 'energy-t' | 'angVel-t';

@Component({
  selector: 'app-graph-view',
  templateUrl: './graph-view.component.html',
})
export class GraphViewComponent implements AfterViewInit {
  data = input.required<GraphDataPoint[]>();
  targetObject = input<PhysicsObject | null>();
  @ViewChild('chart', { static: true }) private chartContainer!: ElementRef;

  activeTab = signal<GraphTab>('x-t');

  constructor() {
    effect(() => {
        const target = this.targetObject();
        // Reset to a default tab when the target object type changes
        if (target) {
            switch(target.type) {
                case 'ball':
                case 'block': this.activeTab.set('x-t'); break;
                case 'spring': this.activeTab.set('length-t'); break;
                case 'rod': this.activeTab.set('angVel-t'); break;
            }
        }
    }, { allowSignalWrites: true });

    effect(() => {
      if (this.chartContainer) {
        this.drawChart();
      }
    });
  }
  
  ngAfterViewInit(): void {
    this.drawChart();
  }

  private drawChart(): void {
    const data = this.data();
    const el = this.chartContainer.nativeElement;
    d3.select(el).select('svg').remove();
    
    if (data.length < 2) return;

    const margin = { top: 20, right: 20, bottom: 30, left: 40 };
    const width = el.clientWidth - margin.left - margin.right;
    const height = el.clientHeight - margin.top - margin.bottom;

    const svg = d3.select(el).append('svg')
      .attr('width', width + margin.left + margin.right)
      .attr('height', height + margin.top + margin.bottom)
      .append('g')
      .attr('transform', `translate(${margin.left},${margin.top})`);
      
    svg.append("defs").append("clipPath")
      .attr("id", "clip")
      .append("rect")
      .attr("width", width)
      .attr("height", height);

    const chartContent = svg.append("g").attr("clip-path", "url(#clip)");

    const tab = this.activeTab();

    if (tab === 'x-t' || tab === 'v-t') {
      const isPosition = tab === 'x-t';
      const y1Key = isPosition ? 'x' : 'vx';
      const y2Key = isPosition ? 'y' : 'vy';
      const yLabel = isPosition ? '位置 (m)' : '速度 (m/s)';

      const xScale = d3.scaleLinear()
        .domain(d3.extent(data, (d: any) => d.t))
        .range([0, width]);

      const yDomain = d3.extent(data.flatMap((d: any) => [d[y1Key], d[y2Key]]));

      const yScale = d3.scaleLinear()
        .domain(yDomain)
        .range([height, 0]).nice();
      
      const line1 = d3.line()
        .x((d: any) => xScale(d.t))
        .y((d: any) => yScale(d[y1Key]));
      
      const line2 = d3.line()
        .x((d: any) => xScale(d.t))
        .y((d: any) => yScale(d[y2Key]));
      
      svg.append('g')
        .attr('transform', `translate(0,${height})`)
        .call(d3.axisBottom(xScale).ticks(5))
        .attr('color', '#9ca3af');
      svg.append('g')
        .call(d3.axisLeft(yScale).ticks(5))
        .attr('color', '#9ca3af');
        
      chartContent.append('path')
        .datum(data)
        .attr('fill', 'none')
        .attr('stroke', '#38bdf8') // light blue
        .attr('stroke-width', 2)
        .attr('d', line1);

      chartContent.append('path')
        .datum(data)
        .attr('fill', 'none')
        .attr('stroke', '#f472b6') // pink
        .attr('stroke-width', 2)
        .attr('d', line2);
      
      svg.append("text").attr("x", 5).attr("y", -5).text(yLabel).attr("fill", "#cbd5e1").style("font-size", "12px");
      svg.append("text").attr("x", width).attr("y", height - 5).text("时间 (s)").attr("fill", "#cbd5e1").style("font-size", "12px").attr("text-anchor", "end");

    } else if (tab === 'x-y') {
        const xDomain = d3.extent(data, (d: any) => d.x) as [number, number];
        const yDomain = d3.extent(data, (d: any) => d.y) as [number, number];

        const rangeX = xDomain[1] - xDomain[0];
        const rangeY = yDomain[1] - yDomain[0];
        const aRatio = width / height;
        const dRatio = rangeX / rangeY;
        
        let finalXRange, finalYRange;
        if (aRatio > dRatio) {
            const newRangeX = rangeY * aRatio;
            const diff = newRangeX - rangeX;
            finalXRange = [xDomain[0] - diff / 2, xDomain[1] + diff / 2];
            finalYRange = yDomain;
        } else {
            const newRangeY = rangeX / aRatio;
            const diff = newRangeY - rangeY;
            finalYRange = [yDomain[0] - diff / 2, yDomain[1] + diff / 2];
            finalXRange = xDomain;
        }

      const xScale = d3.scaleLinear().domain(finalXRange).range([0, width]);
      const yScale = d3.scaleLinear().domain(finalYRange).range([height, 0]);

      const line = d3.line()
        .x((d: any) => xScale(d.x))
        .y((d: any) => yScale(d.y));

      svg.append('g').attr('transform', `translate(0,${height})`).call(d3.axisBottom(xScale).ticks(5)).attr('color', '#9ca3af');
      svg.append('g').call(d3.axisLeft(yScale).ticks(5)).attr('color', '#9ca3af');
        
      chartContent.append('path')
        .datum(data)
        .attr('fill', 'none')
        .attr('stroke', '#6ee7b7') // emerald
        .attr('stroke-width', 2)
        .attr('d', line);
      
      svg.append("text").attr("x", 5).attr("y", -5).text("Y轴位置 (m)").attr("fill", "#cbd5e1").style("font-size", "12px");
      svg.append("text").attr("x", width).attr("y", height - 5).text("X轴位置 (m)").attr("fill", "#cbd5e1").style("font-size", "12px").attr("text-anchor", "end");
    } else if (tab === 'length-t' || tab === 'energy-t' || tab === 'angVel-t' || tab === 'ke-t') {
      const isLength = tab === 'length-t';
      const isEnergy = tab === 'energy-t';
      const isAngVel = tab === 'angVel-t';
      const isKE = tab === 'ke-t';

      const yKey = isLength ? 'length' : isEnergy ? 'potentialEnergy' : isAngVel ? 'angularVelocity' : 'kineticEnergy';
      const yLabel = isLength ? '长度 (m)' : isEnergy ? '势能 (J)' : isAngVel ? '角速度 (rad/s)' : '动能 (J)';
      const color = isLength ? '#f6e05e' : isEnergy ? '#c084fc' : isAngVel ? '#f87171' : '#f97316';

      const xScale = d3.scaleLinear().domain(d3.extent(data, (d:any) => d.t)).range([0, width]);
      const yScale = d3.scaleLinear().domain(d3.extent(data, (d:any) => d[yKey])).range([height, 0]).nice();
      
      const line = d3.line().x((d: any) => xScale(d.t)).y((d: any) => yScale(d[yKey]));
      
      svg.append('g').attr('transform', `translate(0,${height})`).call(d3.axisBottom(xScale).ticks(5)).attr('color', '#9ca3af');
      svg.append('g').call(d3.axisLeft(yScale).ticks(5)).attr('color', '#9ca3af');
        
      chartContent.append('path')
        .datum(data)
        .attr('fill', 'none')
        .attr('stroke', color)
        .attr('stroke-width', 2)
        .attr('d', line);
      
      svg.append("text").attr("x", 5).attr("y", -5).text(yLabel).attr("fill", "#cbd5e1").style("font-size", "12px");
      svg.append("text").attr("x", width).attr("y", height - 5).text("时间 (s)").attr("fill", "#cbd5e1").style("font-size", "12px").attr("text-anchor", "end");
    }
  }

  selectTab(tab: GraphTab): void {
    this.activeTab.set(tab);
  }
}