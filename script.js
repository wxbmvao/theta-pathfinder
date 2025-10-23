const workerCode = `
// priority queue
class MinHeap {
  constructor(){ this.nodes = []; }
  push(item, priority){
    this.nodes.push({item,priority});
    this._siftUp(this.nodes.length-1);
  }
  pop(){
    if(!this.nodes.length) return null;
    const top = this.nodes[0];
    const last = this.nodes.pop();
    if(this.nodes.length) { this.nodes[0] = last; this._siftDown(0); }
    return top.item;
  }
  _siftUp(i){
    while(i>0){
      const p = Math.floor((i-1)/2);
      if(this.nodes[i].priority >= this.nodes[p].priority) break;
      [this.nodes[i], this.nodes[p]] = [this.nodes[p], this.nodes[i]];
      i = p;
    }
  }
  _siftDown(i){
    while(true){
      const l = i*2+1, r = i*2+2, n = this.nodes.length;
      let smallest = i;
      if(l<n && this.nodes[l].priority < this.nodes[smallest].priority) smallest = l;
      if(r<n && this.nodes[r].priority < this.nodes[smallest].priority) smallest = r;
      if(smallest===i) break;
      [this.nodes[i], this.nodes[smallest]] = [this.nodes[smallest], this.nodes[i]];
      i = smallest;
    }
  }
  isEmpty(){ return this.nodes.length===0; }
}

// bresenham-based line-of-sight check on grid
function lineOfSight(grid, w, h, x0, y0, x1, y1) {
  // check cells along line from (x0,y0) to (x1,y1) inclusive
  let dx = Math.abs(x1 - x0);
  let dy = Math.abs(y1 - y0);
  let sx = (x0 < x1) ? 1 : -1;
  let sy = (y0 < y1) ? 1 : -1;
  let err = dx - dy;

  let x = x0, y = y0;
  while (true) {
    if(x<0||x>=w||y<0||y>=h) return false;
    if(grid[y*w + x]) return false; // blocked
    if (x === x1 && y === y1) break;
    let e2 = err * 2;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx) { err += dx; y += sy; }
  }
  return true;
}

// theta* implementation on a grid
function thetaStar(grid, w, h, start, target) {
  const idx = (x,y) => y*w + x;
  const inBounds = (x,y) => x>=0 && x<w && y>=0 && y<h;
  const neighbors = (x,y) => {
    const out = [];
    for(let dy=-1; dy<=1; dy++){
      for(let dx=-1; dx<=1; dx++){
        if(dx===0 && dy===0) continue;
        const nx = x+dx, ny = y+dy;
        if(inBounds(nx,ny) && !grid[idx(nx,ny)]) out.push([nx,ny]);
      }
    }
    return out;
  };
  const euclid = (ax,ay,bx,by) => Math.hypot(ax-bx, ay-by);

  const sidx = idx(start.x, start.y), gidx = idx(target.x, target.y);
  const gScore = new Float64Array(w*h);
  for(let i=0;i<w*h;i++) gScore[i] = Infinity;
  const parent = new Int32Array(w*h);
  for(let i=0;i<w*h;i++) parent[i] = -1;
  const open = new MinHeap();
  const closed = new Uint8Array(w*h);

  gScore[sidx] = 0;
  parent[sidx] = sidx; // parent of start is itself
  const fstart = euclid(start.x,start.y,target.x,target.y);
  open.push(sidx, fstart);

  while(!open.isEmpty()) {
    const current = open.pop();
    if(closed[current]) continue;
    closed[current] = 1;
    if(current === gidx) break;

    const cx = current % w, cy = Math.floor(current / w);

    const neighs = neighbors(cx, cy);
    for (const [nx, ny] of neighs) {
      const nidx = idx(nx, ny);
      // theta* improvement, try to connect neighbor to parent(current) if LOS
      const parentOfCur = parent[current];
      const px = parentOfCur % w, py = Math.floor(parentOfCur / w);

      let tentative_g;
      if (lineOfSight(grid, w, h, px, py, nx, ny)) {
        // path from parent(current) to neighbor
        tentative_g = gScore[parentOfCur] + euclid(px,py,nx,ny);
        if (tentative_g < gScore[nidx]) {
          gScore[nidx] = tentative_g;
          parent[nidx] = parentOfCur;
          const f = tentative_g + euclid(nx,ny,target.x,target.y);
          open.push(nidx, f);
        }
      } else {
        // fallback, connect from current
        tentative_g = gScore[current] + euclid(cx,cy,nx,ny);
        if (tentative_g < gScore[nidx]) {
          gScore[nidx] = tentative_g;
          parent[nidx] = current;
          const f = tentative_g + euclid(nx,ny,target.x,target.y);
          open.push(nidx, f);
        }
      }
    }
  }

  // reconstruct path
  if(parent[gidx] === -1) return null; // no path
  const path = [];
  let cur = gidx;
  while (cur !== parent[cur]) {
    const x = cur % w, y = Math.floor(cur / w);
    path.push({x,y});
    cur = parent[cur];
  }
  // add start
  const sx0 = cur % w, sy0 = Math.floor(cur / w);
  path.push({x: sx0, y: sy0});
  path.reverse();
  return path;
}

self.onmessage = function(e) {
  const {grid, w, h, start, target} = e.data;
  // run theta*
  const startTime = performance.now();
  // quick checks
  if(start.x < 0 || start.x >= w || start.y < 0 || start.y >= h ||
     target.x < 0 || target.x >= w || target.y < 0 || target.y >= h) {
    self.postMessage({status:'error', message:'start/target out of bounds'});
    return;
  }
  if(grid[start.y*w + start.x] || grid[target.y*w + target.x]) {
    self.postMessage({status:'error', message:'start or target inside obstacle'});
    return;
  }
  const path = thetaStar(grid, w, h, start, target);
  const elapsed = performance.now() - startTime;
  self.postMessage({status:'ok', path, timeMs:elapsed});
};
`;

// blob
const blob = new Blob([workerCode], { type: 'application/javascript' });
const workerURL = URL.createObjectURL(blob);
const worker = new Worker(workerURL);

// ui + grid
const canvas = document.getElementById('c');
const stats = document.getElementById('status');
const gwInput = document.getElementById('gridW');
const ghInput = document.getElementById('gridH');
const csInput = document.getElementById('cellSize');
const opInput = document.getElementById('obstProb');
const regenBtn = document.getElementById('regen');

let gridW = parseInt(gwInput.value, 10);
let gridH = parseInt(ghInput.value, 10);
let cellSize = parseInt(csInput.value, 10);
let coverage = parseFloat(opInput.value);

let grid = null; // Uint8Array (0 = free, 1 = blocked)
let start = { x: 2, y: 2 };
let target = { x: gridW - 3, y: gridH - 3 };
let path = null;

function initGrid() {
  gridW = parseInt(gwInput.value, 10);
  gridH = parseInt(ghInput.value, 10);
  cellSize = parseInt(csInput.value, 10);
  coverage = parseFloat(opInput.value);

  canvas.width = gridW * cellSize;
  canvas.height = gridH * cellSize;
  grid = new Uint8Array(gridW * gridH);
  // random obstacles, add rectangles too for variety
  for (let y = 0; y < gridH; y++) {
    for (let x = 0; x < gridW; x++) {
      grid[y * gridW + x] = Math.random() < coverage ? 1 : 0;
    }
  }
  // add some random rectangular obstacles
  for (let i = 0; i < Math.round((gridW * gridH) / 400); i++) {
    const rw = 2 + Math.floor(Math.random() * 10);
    const rh = 2 + Math.floor(Math.random() * 6);
    const rx = Math.floor(Math.random() * (gridW - rw));
    const ry = Math.floor(Math.random() * (gridH - rh));
    for (let yy = 0; yy < rh; yy++)
      for (let xx = 0; xx < rw; xx++) grid[(ry + yy) * gridW + (rx + xx)] = 1;
  }

  // ensure a border is free so path can actually be visible if outside
  // leave as is
  // default start/target
  start = {
    x: Math.max(1, Math.floor(gridW * 0.05)),
    y: Math.max(1, Math.floor(gridH * 0.05)),
  };
  target = {
    x: Math.min(gridW - 2, Math.floor(gridW * 0.9)),
    y: Math.min(gridH - 2, Math.floor(gridH * 0.85)),
  };
  // ensure free
  grid[start.y * gridW + start.x] = 0;
  grid[target.y * gridW + target.x] = 0;
}

function draw() {
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // grid
  for (let y = 0; y < gridH; y++) {
    for (let x = 0; x < gridW; x++) {
      const idx = y * gridW + x;
      ctx.fillStyle = grid[idx] ? '#313d3a' : '#f9fbfa';
      ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
    }
  }

  // path line
  if (path && path.length) {
    ctx.beginPath();
    for (let i = 0; i < path.length; i++) {
      const p = path[i];
      const cx = (p.x + 0.5) * cellSize;
      const cy = (p.y + 0.5) * cellSize;
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    }
    ctx.strokeStyle = '#a299bd';
    ctx.lineWidth = Math.max(1.3, cellSize / 4.7);
    ctx.lineJoin = 'round';
    ctx.stroke();

    // path dots
    ctx.fillStyle = 'hotpink';
    for (let i = 1; i < path.length - 1; i++) {
      const p = path[i];
      ctx.beginPath();
      ctx.arc(
        (p.x + 0.5) * cellSize,
        (p.y + 0.5) * cellSize,
        Math.max(1.3, cellSize / 3.5),
        0,
        Math.PI * 2
      );
      ctx.fill();
    }
  }

  // start point
  ctx.fillStyle = '#2B7FFF';
  ctx.beginPath();
  ctx.arc(
    (start.x + 0.5) * cellSize,
    (start.y + 0.5) * cellSize,
    Math.max(3, cellSize * 0.4),
    0,
    Math.PI * 2
  );
  ctx.fill();

  // target point
  ctx.fillStyle = '#2AA63E';
  ctx.beginPath();
  ctx.arc(
    (target.x + 0.5) * cellSize,
    (target.y + 0.5) * cellSize,
    Math.max(3, cellSize * 0.4),
    0,
    Math.PI * 2
  );
  ctx.fill();

  // gridlines
  ctx.strokeStyle = '#d5e0de';
  ctx.lineWidth = 0.4;
  for (let x = 0; x <= gridW; x++) {
    ctx.beginPath();
    ctx.moveTo(x * cellSize, 0);
    ctx.lineTo(x * cellSize, canvas.height);
    ctx.stroke();
  }
  for (let y = 0; y <= gridH; y++) {
    ctx.beginPath();
    ctx.moveTo(0, y * cellSize);
    ctx.lineTo(canvas.width, y * cellSize);
    ctx.stroke();
  }
}

function runWorker() {
  stats.textContent = 'computing...';
  // send typed array buffer copy
  worker.postMessage({ grid: grid, w: gridW, h: gridH, start, target });
}

worker.onmessage = function (e) {
  const data = e.data;
  if (data.status === 'error') {
    stats.textContent = 'error: ' + data.message;
    path = null;
    draw();
    return;
  }
  path = data.path;
  stats.innerHTML =
    `<br />generation time ` +
    data.timeMs.toFixed(1) +
    ` ms <br /> path length: ` +
    (path ? path.length : 0);
  draw();
};

// canvas click to set start/target
let clickMode = 0; // 0 = set start, 1 = set target
canvas.addEventListener('click', (ev) => {
  const rect = canvas.getBoundingClientRect();
  const x = Math.floor((ev.clientX - rect.left) / cellSize);
  const y = Math.floor((ev.clientY - rect.top) / cellSize);
  if (x < 0 || x >= gridW || y < 0 || y >= gridH) return;
  if (grid[y * gridW + x]) return; // no picking inside obstacle
  if (ev.shiftKey) {
    // shift+click toggles obstacle
    grid[y * gridW + x] = grid[y * gridW + x] ? 0 : 1;
    draw();
    return;
  }
  if (ev.button === 0) {
    // left click sets start
    start = { x, y };
  } else {
    target = { x, y };
  }
  runWorker();
});

// regen button
regenBtn.addEventListener('click', () => {
  initGrid();
  runWorker();
  draw();
});

// init
initGrid();
draw();
runWorker();
