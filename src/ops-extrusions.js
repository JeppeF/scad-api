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

function clamp (value, min, max) {
  return Math.min(Math.max(value, min), max)
}

const toPoints = cag => {
  let points
  if ('sides' in cag) {
    console.log('foo')
    points = []
    cag.sides.forEach(side =>{
      points.push([side.vertex0.pos.x, side.vertex0.pos.y])
      points.push([side.vertex1.pos.x, side.vertex1.pos.y])
    })
    //cag.sides.map(side => [side.vertex0.pos.x, side.vertex0.pos.y])
    //, side.vertex1.pos.x, side.vertex1.pos.y])
    // due to the logic of CAG.fromPoints()
    // move the first point to the last
    /*if (points.length > 0) {
      points.push(points.shift())
    }*/
  } else if ('points' in cag) {
    points = cag.points.map(p => ([p.x, p.y]))
  }

  return points
}

const degToRad = deg => (Math.PI / 180) * deg

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

  // angle = clamp(angle, 0, Math.PI * 2) // limit angle

  if (arguments.length < 2) { // FIXME: what the hell ??? just put params second !
    baseShape = params
  }
  if (fn < 1) fn = 1
  const segments = fn // for clarity

  let polygons = []

  // convert baseshape to just an array of points, easier to deal with
  const shapePoints = toPoints(baseShape)
  // determine if the rotate_extrude can be computed in the first place
  // ie all the points have to be either x > 0 or x < 0

  // TODO : for the future : generic solution to always have a valid solid, even if points go beyond x/ -x
  // 1. split points up between all those on the 'left' side of the axis (x<0) & those on the 'righ' (x>0)
  // 2. for each set of points do the extrusion operation IN OPOSITE DIRECTIONS
  // 3. union the two resulting solids

  console.log('shapePoints', shapePoints, baseShape.sides)
  const flipped = angle > 0
  // for each of the intermediary steps in the extrusion
  for (let i = 1; i < segments + 1; i++) {
    // o.{x,y} -> rotate([0,0,i:0..360], obj->{o.x,0,o.y})
    const curAngle = degToRad(i / segments * angle)// startAngle + i * (1.0 / segments) * angle
    let sin = Math.sin(curAngle)
    let cos = Math.cos(curAngle)

    const prevAngle = degToRad((i - 1) / segments * angle)// startAngle + (i - 1) * (1.0 / segments) * angle
    let prevSin = Math.sin(prevAngle)
    let prevCos = Math.cos(prevAngle)

    prevSin = prevCos = sin = cos = 1

    // console.log('shapePoints', shapePoints)
    for (let j = 0; j < shapePoints.length - 1; j++) {
      // 2 points of a side
      const curPoint = shapePoints[j]
      const nextPoint = shapePoints[j + 1]

      let prevMatrix = CSG.Matrix4x4.rotationZ((i - 1) / segments * angle)
      let curMatrix = CSG.Matrix4x4.rotationZ((i) / segments * angle)

      const pointA = rightMultiply1x3VectorSimple(prevMatrix, [curPoint[0], 0, curPoint[1]])
      const pointAP = rightMultiply1x3VectorSimple(curMatrix, [curPoint[0], 0, curPoint[1]])

      const pointB = rightMultiply1x3VectorSimple(prevMatrix, [nextPoint[0], 0, nextPoint[1]])
      const pointBP = rightMultiply1x3VectorSimple(curMatrix, [nextPoint[0], 0, nextPoint[1]])
      const tolerance = 0.001

      console.log(`point ${j} edge connecting ${j} to ${j + 1}`)
      let overlappingPoints = false
      if (Math.abs(pointA[0] - pointAP[0]) < tolerance && Math.abs(pointB[1] - pointBP[1]) < tolerance) {
        console.log('identical / overlapping points (from current angle and next one), what now ?')
        // console.log('at point index', j, 'pointA', pointA[0], pointA[2], 'pointAP', pointAP[0], pointAP[2])
        // continue
        overlappingPoints = true
      }
      //overlappingPoints = false

      // single quad : bad results
      // let polyPoints = [pointA, pointB, pointBP, pointAP]
      // polygons.push(polygonFromPoints(polyPoints))

      if (flipped) {
        // CW
        polygons.push(polygonFromPoints([pointA, pointB, pointBP]))
        if (!overlappingPoints) {
          polygons.push(polygonFromPoints([pointBP, pointAP, pointA]))
        }
      } else {
        // CCW
        if (!overlappingPoints) {
          polygons.push(polygonFromPoints([pointA, pointAP, pointBP]))
        }
        polygons.push(polygonFromPoints([pointBP, pointB, pointA]))
      }
    }
    // if we do not do a full extrusion, we want caps at both ends (closed volume)
    if (Math.abs(angle) < 360) {
      const endMatrix = CSG.Matrix4x4.rotationX(90)
      const endCap = baseShape._toPlanePolygons({flipped: flipped})
        .map(x => x.transform(endMatrix))

      const startMatrix = CSG.Matrix4x4.rotationX(90).multiply(
        CSG.Matrix4x4.rotationZ(-angle)
      )
      const startCap = baseShape._toPlanePolygons({flipped: !flipped})
        .map(x => x.transform(startMatrix))
      polygons = polygons.concat(endCap).concat(startCap)
    }

    /* const pointA = rightMultiply1x3VectorSimple(prevMatrix, [curPoint[0] * prevSin, 0, curPoint[1] * prevCos])
      const pointAP = rightMultiply1x3VectorSimple(curMatrix, [curPoint[0] * sin, 0, curPoint[1] * cos])

      const pointB = rightMultiply1x3VectorSimple(prevMatrix, [nextPoint[0] * prevSin, 0, nextPoint[1] * prevCos])
      const pointBP = rightMultiply1x3VectorSimple(curMatrix, [nextPoint[0] * sin, 0, nextPoint[1] * cos]) */

    /*
    // for each side of the 2d shape
    for (let j = 0; j < baseShape.sides.length; j++) {
      // has o.sides[j].vertex{0,1}.pos (only x,y)
      // TODO : check if points of the baseShape are in the valid 'quadrant' ie: no point beyond 0 , otherwise you
      // get self overlapping shapes

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
      polygons.push(polygonFromPoints(stepPoints))
    } */
  }
  return CSG.fromPolygons(polygons).reTesselated().canonicalized()
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
