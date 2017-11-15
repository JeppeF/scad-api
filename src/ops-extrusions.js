const { CSG } = require('@jscad/csg')
// -- 2D to 3D primitives (OpenSCAD like notion)

function linear_extrude (p, s) {
  // console.log("linear_extrude() not yet implemented")
  // return
  let h = 1
  let off = 0
  let twist = 0
  let slices = 10
  /* convexity = 10, */

  if (p.height) h = p.height
  // if(p.convexity) convexity = p.convexity      // abandoned
  if (p.twist) twist = p.twist
  if (p.slices) slices = p.slices
  var o = s.extrude({offset: [0, 0, h], twistangle: twist, twiststeps: slices})
  if (p.center === true) {
    var b = [ ]
    b = o.getBounds() // b[0] = min, b[1] = max
    off = b[1].plus(b[0])
    off = off.times(-0.5)
    o = o.translate(off)
  }
  return o
}

// FIXME: this is to have more readable/less extremely verbose code below
const vertexFromVectorArray = array => {
  return new CSG.Vertex(new CSG.Vector3D(array))
}

const polygonFromPoints = points => {
  // EEK talk about wrapping wrappers !
  const vertices = points.map(point => new CSG.Vertex(new CSG.Vector3D(point)))
  return new CSG.Polygon(vertices)
}

// Simplified, array vector rightMultiply1x3Vector
const rightMultiply1x3VectorSimple = (matrix, vector) => {
  const [v0, v1, v2] = vector
  const v3 = 1
  let x = v0 * matrix.elements[0] + v1 * matrix.elements[1] + v2 * matrix.elements[2] + v3 * matrix.elements[3]
  let y = v0 * matrix.elements[4] + v1 * matrix.elements[5] + v2 * matrix.elements[6] + v3 * matrix.elements[7]
  let z = v0 * matrix.elements[8] + v1 * matrix.elements[9] + v2 * matrix.elements[10] + v3 * matrix.elements[11]
  let w = v0 * matrix.elements[12] + v1 * matrix.elements[13] + v2 * matrix.elements[14] + v3 * matrix.elements[15]

  // scale such that fourth element becomes 1:
  if (w !== 1) {
    const invw = 1.0 / w
    x *= invw
    y *= invw
    z *= invw
  }
  return [x, y, z]
}

/** rotate extrude / revolve
 * @param {Object} [options] - options for construction
 * @param {Integer} [options.fn=1] - resolution/number of segments of the extrusion
 * @returns {CSG} new extruded shape
 *
 * @example
 * let revolved = rotate_extrude({fn: 10}, square())
 */
function rotate_extrude (params, baseShape) {
  const defaults = {
    fn: 32,
    startAngle: 0,
    angle: 360
  }
  params = Object.assign({}, defaults, params)
  let {fn, startAngle, angle} = params

  // angle = clamp(angle, 0, Math.PI * 2)
  if (arguments.length < 2) { // FIXME: what the hell ??? just put params second !
    baseShape = params
  }
  if (fn < 3) fn = 3

  let points = []

  // for each of the intermediary steps in the extrusion
  for (let i = 0; i < fn; i++) {
    // o.{x,y} -> rotate([0,0,i:0..360], obj->{o.x,0,o.y})
    for (let j = 0; j < baseShape.sides.length; j++) {
      // has o.sides[j].vertex{0,1}.pos (only x,y)
      let stepPoints = []
      let matrix

      matrix = CSG.Matrix4x4.rotationZ(i / fn * angle)
      stepPoints.push(
        rightMultiply1x3VectorSimple(matrix, [baseShape.sides[j].vertex0.pos.x, 0, baseShape.sides[j].vertex0.pos.y])
      )

      stepPoints.push(
        rightMultiply1x3VectorSimple(matrix, [baseShape.sides[j].vertex1.pos.x, 0, baseShape.sides[j].vertex1.pos.y])
      )

      matrix = CSG.Matrix4x4.rotationZ((i + 1) / fn * angle)

      stepPoints.push(
        rightMultiply1x3VectorSimple(matrix, [baseShape.sides[j].vertex1.pos.x, 0, baseShape.sides[j].vertex1.pos.y])
      )

      stepPoints.push(
        rightMultiply1x3VectorSimple(matrix, [baseShape.sides[j].vertex0.pos.x, 0, baseShape.sides[j].vertex0.pos.y])
      )
      // we make a square polygon (instead of 2 triangles)
      points.push(polygonFromPoints(stepPoints))
    }
  }
  return CSG.fromPolygons(points)
}

function rectangular_extrude (pa, p) {
  let w = 1
  let h = 1
  let fn = 8
  let closed = false
  let round = true
  if (p) {
    if (p.w) w = p.w
    if (p.h) h = p.h
    if (p.fn) fn = p.fn
    if (p.closed !== undefined) closed = p.closed
    if (p.round !== undefined) round = p.round
  }
  return new CSG.Path2D(pa, closed).rectangularExtrude(w, h, fn, round)
}

module.exports = {
  linear_extrude,
  rotate_extrude,
  rectangular_extrude
}
