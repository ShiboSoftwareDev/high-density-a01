import { test, expect } from "bun:test"
import "bun-match-svg"
import "graphics-debug/matcher"
import { HighDensitySolverA01 } from "../../../lib/HighDensitySolverA01/HighDensitySolverA01"
import sample001 from "./sample001.json"

test("sample001 initial visualization", async () => {
  const solver = new HighDensitySolverA01({
    nodeWithPortPoints: sample001,
    cellSizeMm: 0.05,
    viaDiameter: 0.3,
  })

  solver.setup()

  const graphics = solver.visualize()

  await expect(graphics).toMatchGraphicsSvg(import.meta.path)
})
