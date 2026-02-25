import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import { HighDensitySolverA01 } from "../../lib/HighDensitySolverA01/HighDensitySolverA01"
import sample001 from "../../tests/dataset01/sample001/sample001.json"

const { width, height } = sample001
const borderMargin = 2

export default () => (
  <GenericSolverDebugger
    createSolver={() => {
      const solver = new HighDensitySolverA01({
        nodeWithPortPoints: sample001,
        cellSizeMm: 0.5,
        viaDiameter: 0.3,
      })
      return solver
    }}
  />
)
