import { BaseSolver } from "@tscircuit/solver-utils"
import type { HighDensityIntraNodeRoute, NodeWithPortPoints } from "../types"

type ConnectionName = string
type CellKey = string

interface HyperParameters {
  shuffleSeed: number
  ripCost: number
  ripTracePenalty: number
  ripViaPenalty: number
  viaBaseCost: number
}

interface HighDensitySolverA01Props {
  nodeWithPortPoints: NodeWithPortPoints
  cellSizeMm: number
  viaDiameter: number
  traceThickness?: number
  traceMargin?: number
  hyperParameters?: Partial<HyperParameters>
  initialPenaltyFn?: (params: {
    x: number
    y: number
    px: number
    py: number
    row: number
    col: number
  }) => number
}

interface GridCell {
  row: number
  col: number
  z: number
  x: number
  y: number
}

interface CandidateCell {
  cell: GridCell
  g: number
  f: number
  parent: CandidateCell | null
  rippedTraces: Set<ConnectionName>
}

interface Connection {
  connectionName: ConnectionName
  start: GridCell
  end: GridCell
}

export class HighDensitySolverA01 extends BaseSolver {
  nodeWithPortPoints: NodeWithPortPoints
  cellSizeMm: number
  viaDiameter: number
  traceThickness: number
  traceMargin: number
  hyperParameters: HyperParameters
  initialPenaltyFn?: HighDensitySolverA01Props["initialPenaltyFn"]

  // Grid dimensions
  rows!: number
  cols!: number
  layers!: number
  gridOrigin!: { x: number; y: number }

  // Z-layer mapping: actual z value <-> layer index
  availableZ!: number[]
  zToLayer!: Map<number, number>
  layerToZ!: Map<number, number>

  // Penalty map: [row][col] -> additional traversal cost
  penaltyMap!: number[][]

  // Used cells: [z][row][col] -> connectionName or null
  usedCells!: (ConnectionName | null)[][][]

  // Connection queues
  unsolvedConnections!: Connection[]
  solvedConnectionsMap!: Map<ConnectionName, HighDensityIntraNodeRoute>

  // Current A* state for the active connection
  activeConnection: Connection | null = null
  openSet!: CandidateCell[]
  closedSet!: Set<CellKey>

  constructor(props: HighDensitySolverA01Props) {
    super()
    this.nodeWithPortPoints = props.nodeWithPortPoints
    this.cellSizeMm = props.cellSizeMm
    this.viaDiameter = props.viaDiameter
    this.traceThickness = props.traceThickness ?? 0.1
    this.traceMargin = props.traceMargin ?? 0.15
    this.hyperParameters = {
      shuffleSeed: 0,
      ripCost: 10,
      ripTracePenalty: 0.5,
      ripViaPenalty: 0.75,
      viaBaseCost: 0.1,
      ...props.hyperParameters,
    }
    this.initialPenaltyFn = props.initialPenaltyFn
  }

  override _setup(): void {
    const { nodeWithPortPoints, cellSizeMm } = this
    const { width, height, center } = nodeWithPortPoints
    // Derive available z layers from port points if not provided
    this.availableZ = nodeWithPortPoints.availableZ ?? [
      ...new Set(nodeWithPortPoints.portPoints.map((pp) => pp.z)),
    ].sort((a, b) => a - b)

    this.zToLayer = new Map()
    this.layerToZ = new Map()
    for (let i = 0; i < this.availableZ.length; i++) {
      const z = this.availableZ[i]!
      this.zToLayer.set(z, i)
      this.layerToZ.set(i, z)
    }

    this.rows = Math.ceil(height / cellSizeMm)
    this.cols = Math.ceil(width / cellSizeMm)
    this.layers = this.availableZ.length
    this.gridOrigin = {
      x: center.x - width / 2,
      y: center.y - height / 2,
    }

    // Initialize penalty map
    this.penaltyMap = Array.from({ length: this.rows }, (_, row) =>
      Array.from({ length: this.cols }, (_, col) => {
        if (!this.initialPenaltyFn) return 0
        const x = this.gridOrigin.x + (col + 0.5) * cellSizeMm
        const y = this.gridOrigin.y + (row + 0.5) * cellSizeMm
        const px = (col + 0.5) / this.cols
        const py = (row + 0.5) / this.rows
        return this.initialPenaltyFn({ x, y, px, py, row, col })
      }),
    )

    // Initialize used cells: [z][row][col]
    this.usedCells = Array.from({ length: this.layers }, () =>
      Array.from({ length: this.rows }, () =>
        Array<ConnectionName | null>(this.cols).fill(null),
      ),
    )

    // Build connections from port points
    this.unsolvedConnections = this.buildConnectionsFromPortPoints()
    this.solvedConnectionsMap = new Map()

    // Shuffle based on seed
    this.shuffleConnections()

    // Reset A* state
    this.activeConnection = null
    this.openSet = []
    this.closedSet = new Set()
  }

  override _step(): void {
    // 1. If no active connection, dequeue the next unsolved one
    if (!this.activeConnection) {
      if (this.unsolvedConnections.length === 0) {
        this.solved = true
        return
      }

      const next = this.unsolvedConnections.shift()
      if (!next) {
        this.solved = true
        return
      }

      this.activeConnection = next
      this.openSet = [
        {
          cell: this.activeConnection.start,
          g: 0,
          f: this.computeH(
            this.activeConnection.start,
            this.activeConnection.end,
          ),
          parent: null,
          rippedTraces: new Set(),
        },
      ]
      this.closedSet = new Set()
      return
    }

    // 2. If open set is empty, this connection failed
    if (this.openSet.length === 0) {
      this.error = `No path found for ${this.activeConnection.connectionName}`
      this.failed = true
      return
    }

    // 3. Dequeue best candidate (lowest f)
    this.openSet.sort((a, b) => a.f - b.f)
    const current = this.openSet.shift()!

    const { cell } = current
    const cellKey = this.getCellKey(cell)

    // Skip if already visited
    if (this.closedSet.has(cellKey)) return
    this.closedSet.add(cellKey)

    // 4. Check end condition
    if (
      cell.row === this.activeConnection.end.row &&
      cell.col === this.activeConnection.end.col &&
      cell.z === this.activeConnection.end.z
    ) {
      this.finalizeRoute(current)
      this.activeConnection = null
      return
    }

    // 5. Expand neighbors (8 directions + via)
    for (const neighbor of this.getNeighbors(cell)) {
      if (this.closedSet.has(this.getCellKey(neighbor))) continue

      const g = current.g + this.computeG(cell, neighbor, current.rippedTraces)
      const f = g + this.computeH(neighbor, this.activeConnection.end)

      const rippedTraces = new Set(current.rippedTraces)
      const occupant =
        this.usedCells[neighbor.z]?.[neighbor.row]?.[neighbor.col]
      if (occupant && occupant !== this.activeConnection.connectionName) {
        rippedTraces.add(occupant)
      }

      this.openSet.push({
        cell: neighbor,
        g,
        f,
        parent: current,
        rippedTraces,
      })
    }
  }

  override visualize() {
    const LAYER_COLORS = ["red", "blue", "orange", "green"]

    const points: Array<{
      x: number
      y: number
      color?: string
      label?: string
    }> = []
    const lines: Array<{
      points: Array<{ x: number; y: number }>
      strokeColor?: string
      strokeWidth?: number
    }> = []
    const rects: Array<{
      center: { x: number; y: number }
      width: number
      height: number
      stroke?: string
    }> = []

    // Draw grid bounds
    const { width, height, center } = this.nodeWithPortPoints
    rects.push({
      center: { x: center.x, y: center.y },
      width,
      height,
      stroke: "gray",
    })

    // Draw port points colored by layer
    for (const pp of this.nodeWithPortPoints.portPoints) {
      points.push({
        x: pp.x,
        y: pp.y,
        color: LAYER_COLORS[pp.z] ?? "gray",
        label: pp.connectionName,
      })
    }

    // Draw solved routes, splitting segments by z-layer for correct coloring
    if (this.solvedConnectionsMap) {
      for (const [, route] of this.solvedConnectionsMap) {
        if (route.route.length < 2) continue

        // Split the route into segments of contiguous z values
        let segStart = 0
        for (let i = 1; i < route.route.length; i++) {
          const prev = route.route[i - 1]!
          const curr = route.route[i]!
          if (curr.z !== prev.z) {
            // Emit segment for the previous z
            if (i - segStart >= 2) {
              lines.push({
                points: route.route
                  .slice(segStart, i)
                  .map((p) => ({ x: p.x, y: p.y })),
                strokeColor: LAYER_COLORS[prev.z] ?? "gray",
                strokeWidth: this.traceThickness,
              })
            }
            segStart = i
          }
        }
        // Emit final segment
        if (route.route.length - segStart >= 2) {
          const lastZ = route.route[segStart]!.z
          lines.push({
            points: route.route
              .slice(segStart)
              .map((p) => ({ x: p.x, y: p.y })),
            strokeColor: LAYER_COLORS[lastZ] ?? "gray",
            strokeWidth: this.traceThickness,
          })
        }
      }
    }

    // Draw active A* exploration
    if (this.activeConnection && this.closedSet) {
      for (const key of this.closedSet) {
        const parts = key.split(",")
        const row = Number(parts[1])
        const col = Number(parts[2])
        points.push({
          x: this.gridOrigin.x + (col + 0.5) * this.cellSizeMm,
          y: this.gridOrigin.y + (row + 0.5) * this.cellSizeMm,
          color: "rgba(0,0,255,0.2)",
        })
      }
    }

    return {
      points,
      lines,
      rects,
      coordinateSystem: "cartesian" as const,
      title: `HighDensityA01 [${this.solvedConnectionsMap?.size ?? 0} solved, ${this.unsolvedConnections?.length ?? 0} remaining]`,
    }
  }

  // --- Internal helpers ---

  buildConnectionsFromPortPoints(): Connection[] {
    const byName = new Map<
      ConnectionName,
      NodeWithPortPoints["portPoints"]
    >()
    for (const pp of this.nodeWithPortPoints.portPoints) {
      const name = pp.connectionName
      if (!byName.has(name)) byName.set(name, [])
      byName.get(name)!.push(pp)
    }

    const connections: Connection[] = []
    for (const [name, pts] of byName) {
      if (pts.length < 2) continue
      for (let i = 0; i < pts.length - 1; i++) {
        const start = pts[i]!
        const end = pts[i + 1]!
        connections.push({
          connectionName: name,
          start: this.pointToGridCell(start),
          end: this.pointToGridCell(end),
        })
      }
    }
    return connections
  }

  pointToGridCell(pt: { x: number; y: number; z: number }): GridCell {
    const col = Math.round(
      (pt.x - this.gridOrigin.x) / this.cellSizeMm - 0.5,
    )
    const row = Math.round(
      (pt.y - this.gridOrigin.y) / this.cellSizeMm - 0.5,
    )
    const layerIndex = this.zToLayer.get(pt.z) ?? 0
    return {
      row: Math.max(0, Math.min(this.rows - 1, row)),
      col: Math.max(0, Math.min(this.cols - 1, col)),
      z: layerIndex,
      x: this.gridOrigin.x + (col + 0.5) * this.cellSizeMm,
      y: this.gridOrigin.y + (row + 0.5) * this.cellSizeMm,
    }
  }

  shuffleConnections(): void {
    const seed = this.hyperParameters.shuffleSeed
    let s = seed
    const rng = () => {
      s = (s * 1664525 + 1013904223) & 0xffffffff
      return (s >>> 0) / 0xffffffff
    }
    for (let i = this.unsolvedConnections.length - 1; i > 0; i--) {
      const j = Math.floor(rng() * (i + 1))
      const a = this.unsolvedConnections[i]!
      const b = this.unsolvedConnections[j]!
      this.unsolvedConnections[i] = b
      this.unsolvedConnections[j] = a
    }
  }

  getCellKey(cell: GridCell): CellKey {
    return `${cell.z},${cell.row},${cell.col}`
  }

  computeH(a: GridCell, b: GridCell): number {
    return (
      (Math.abs(a.row - b.row) + Math.abs(a.col - b.col)) * this.cellSizeMm
    )
  }

  computeG(
    from: GridCell,
    to: GridCell,
    rippedTraces: Set<ConnectionName>,
  ): number {
    let cost = 0

    if (from.z !== to.z) {
      // Via transition
      cost += this.hyperParameters.viaBaseCost
    } else {
      // Lateral movement (diagonal = sqrt(2), orthogonal = 1)
      const dr = Math.abs(from.row - to.row)
      const dc = Math.abs(from.col - to.col)
      cost += (dr + dc > 1 ? Math.SQRT2 : 1) * this.cellSizeMm
    }

    // Penalty map
    cost += this.penaltyMap[to.row]?.[to.col] ?? 0

    // Rip cost for occupied cells
    const occupant = this.usedCells[to.z]?.[to.row]?.[to.col]
    if (occupant && occupant !== this.activeConnection?.connectionName) {
      if (!rippedTraces.has(occupant)) {
        cost += this.hyperParameters.ripCost
      }
      cost +=
        from.z !== to.z
          ? this.hyperParameters.ripViaPenalty
          : this.hyperParameters.ripTracePenalty
    }

    return cost
  }

  getNeighbors(cell: GridCell): GridCell[] {
    const neighbors: GridCell[] = []
    const dirs = [
      [-1, -1],
      [-1, 0],
      [-1, 1],
      [0, -1],
      [0, 1],
      [1, -1],
      [1, 0],
      [1, 1],
    ]

    for (const dir of dirs) {
      const dr = dir[0]!
      const dc = dir[1]!
      const row = cell.row + dr
      const col = cell.col + dc
      if (row < 0 || row >= this.rows || col < 0 || col >= this.cols) continue
      neighbors.push({
        row,
        col,
        z: cell.z,
        x: this.gridOrigin.x + (col + 0.5) * this.cellSizeMm,
        y: this.gridOrigin.y + (row + 0.5) * this.cellSizeMm,
      })
    }

    // Via: move to other layers at same position
    for (let z = 0; z < this.layers; z++) {
      if (z === cell.z) continue
      neighbors.push({ ...cell, z })
    }

    return neighbors
  }

  finalizeRoute(candidate: CandidateCell): void {
    // Reconstruct path from candidate chain
    const routePoints: Array<{ x: number; y: number; z: number }> = []
    const vias: Array<{ x: number; y: number }> = []
    let node: CandidateCell | null = candidate

    while (node) {
      routePoints.unshift({
        x: node.cell.x,
        y: node.cell.y,
        z: node.cell.z,
      })
      node = node.parent
    }

    // Detect vias (z-level changes)
    for (let i = 1; i < routePoints.length; i++) {
      const curr = routePoints[i]!
      const prev = routePoints[i - 1]!
      if (curr.z !== prev.z) {
        vias.push({ x: curr.x, y: curr.y })
      }
    }

    const connName = this.activeConnection!.connectionName

    // Rip any traces we displaced
    for (const rippedName of candidate.rippedTraces) {
      this.ripTrace(rippedName)
    }

    // Mark cells as used (including margin cells around the trace)
    const marginCells = Math.ceil(this.traceMargin / this.cellSizeMm)
    for (const pt of routePoints) {
      const centerRow = Math.round(
        (pt.y - this.gridOrigin.y) / this.cellSizeMm - 0.5,
      )
      const centerCol = Math.round(
        (pt.x - this.gridOrigin.x) / this.cellSizeMm - 0.5,
      )
      for (let dr = -marginCells; dr <= marginCells; dr++) {
        for (let dc = -marginCells; dc <= marginCells; dc++) {
          const r = centerRow + dr
          const c = centerCol + dc
          if (r < 0 || r >= this.rows || c < 0 || c >= this.cols) continue
          const layer = this.usedCells[pt.z]
          if (layer) {
            const rowArr = layer[r]
            if (rowArr) {
              rowArr[c] = connName
            }
          }
        }
      }
    }

    // Mark via footprint cells (vias occupy more cells based on viaDiameter)
    const viaRadiusCells = Math.ceil(this.viaDiameter / 2 / this.cellSizeMm)
    for (const via of vias) {
      const viaRow = Math.round(
        (via.y - this.gridOrigin.y) / this.cellSizeMm - 0.5,
      )
      const viaCol = Math.round(
        (via.x - this.gridOrigin.x) / this.cellSizeMm - 0.5,
      )
      for (let z = 0; z < this.layers; z++) {
        for (let dr = -viaRadiusCells; dr <= viaRadiusCells; dr++) {
          for (let dc = -viaRadiusCells; dc <= viaRadiusCells; dc++) {
            const r = viaRow + dr
            const c = viaCol + dc
            if (r < 0 || r >= this.rows || c < 0 || c >= this.cols) continue
            if (dr * dr + dc * dc <= viaRadiusCells * viaRadiusCells) {
              const layer = this.usedCells[z]
              if (layer) {
                const rowArr = layer[r]
                if (rowArr) {
                  rowArr[c] = connName
                }
              }
            }
          }
        }
      }
    }

    // Store solved route (map layer indices back to real z values)
    this.solvedConnectionsMap.set(connName, {
      connectionName: connName,
      traceThickness: this.traceThickness,
      viaDiameter: this.viaDiameter,
      route: routePoints.map((pt) => ({
        x: pt.x,
        y: pt.y,
        z: this.layerToZ.get(pt.z) ?? pt.z,
      })),
      vias,
    })
  }

  ripTrace(connectionName: ConnectionName): void {
    // Remove from usedCells
    for (let z = 0; z < this.layers; z++) {
      for (let row = 0; row < this.rows; row++) {
        for (let col = 0; col < this.cols; col++) {
          const layer = this.usedCells[z]
          if (layer) {
            const rowArr = layer[row]
            if (rowArr && rowArr[col] === connectionName) {
              rowArr[col] = null
            }
          }
        }
      }
    }

    // Move from solved back to unsolved
    const route = this.solvedConnectionsMap.get(connectionName)
    if (route) {
      this.solvedConnectionsMap.delete(connectionName)
      const start = route.route[0]
      const end = route.route[route.route.length - 1]
      if (start && end) {
        this.unsolvedConnections.push({
          connectionName,
          start: this.pointToGridCell(start),
          end: this.pointToGridCell(end),
        })
      }
    }
  }

  override getOutput(): HighDensityIntraNodeRoute[] {
    return Array.from(this.solvedConnectionsMap.values())
  }
}
