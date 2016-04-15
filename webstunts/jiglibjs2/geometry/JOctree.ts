

/// <reference path="../data/OctreeCell.ts"/>

/// <reference path="JIndexedTriangle.ts"/>

module jiglib {

    export class JOctree {
        private _cells: OctreeCell[] = []; // OctreeCell
        private _vertices: Vector3D[] = []; // Vector3D
        private _triangles: JIndexedTriangle[] = [];//null; // JIndexedTriangle
        private _boundingBox = new JAABox(); // JAABox
        private _cellsToTest: number[] = [];//null; // int
        private _testCounter = 0; // int

        constructor() { }

        get_trianglesData() {

            return this._triangles;

        }

        getTriangle(iTriangle) {

            return this._triangles[iTriangle];

        }

        get_verticesData() {

            return this._vertices;

        }

        getVertex(iVertex) {

            return this._vertices[iVertex];

        }

        boundingBox() {

            return this._boundingBox;

        }

        clear() {

            this._cells.length = 0;
            this._vertices.length = 0;
            this._triangles.length = 0;

        }

        addTriangles(vertices: Vector3D[], numVertices: number, triangleVertexIndices: TriangleVertexIndices[], numTriangles: number) {

            this.clear();

            this._vertices = vertices.concat();

            var tiny = JMath3D.NUM_TINY;
            for (let tri of triangleVertexIndices) {
                let i0 = tri.i0;
                let i1 = tri.i1;
                let i2 = tri.i2;

                let dr1 = vertices[i1].subtract(vertices[i0]);
                let dr2 = vertices[i2].subtract(vertices[i0]);
                let N = dr1.crossProduct(dr2);
                let NLen = N.get_length();

                if (NLen > tiny) {
                    let indexedTriangle = new JIndexedTriangle();
                    indexedTriangle.setVertexIndices(i0, i1, i2, this._vertices);
                    this._triangles.push(indexedTriangle);
                }
            }

        }

        buildOctree(maxTrianglesPerCell: number, minCellSize: number) {

            this._boundingBox.clear();

            for (let vt of this._vertices) {
                this._boundingBox.addPoint(vt);
            }

            this._cells.length = 0;
            this._cells.push(new OctreeCell(this._boundingBox));

            var numTriangles = this._triangles.length;
            for (var i = 0; i < numTriangles; i++) {
                this._cells[0].triangleIndices[i] = i;
            }

            var cellsToProcess: number[] = [];
            cellsToProcess.push(0);

            while (cellsToProcess.length != 0) {
                let cellIndex = cellsToProcess.pop();

                if (this._cells[cellIndex].triangleIndices.length <= maxTrianglesPerCell || this._cells[cellIndex].AABox.getRadiusAboutCentre() < minCellSize) {
                    continue;
                }
                for (i = 0; i < OctreeCell.NUM_CHILDREN; i++) {
                    this._cells[cellIndex].childCellIndices[i] = this._cells.length;
                    cellsToProcess.push(this._cells.length);
                    this._cells.push(new OctreeCell(this.createAABox(this._cells[cellIndex].AABox, i)));

                    let childCell = this._cells[this._cells.length - 1];
                    numTriangles = this._cells[cellIndex].triangleIndices.length;
                    for (var j = 0; j < numTriangles; j++) {
                        let iTri = this._cells[cellIndex].triangleIndices[j];
                        if (this.doesTriangleIntersectCell(this._triangles[iTri], childCell)) {
                            childCell.triangleIndices.push(iTri);
                        }
                    }
                }
                this._cells[cellIndex].triangleIndices.length = 0;
            }

        }

        updateTriangles(vertices: Vector3D[]) {

            this._vertices = vertices.concat();

            for (let triangle of this._triangles) {
                triangle.updateVertexIndices(this._vertices);
            }

        }

        getTrianglesIntersectingtAABox(triangles: number[], aabb: JAABox) {

            if (this._cells.length == 0)
                return 0;

            this._cellsToTest.length = 0;
            this._cellsToTest.push(0);

            this.incrementTestCounter();


            while (this._cellsToTest.length != 0) {
                let cellIndex = this._cellsToTest.pop();

                let cell = this._cells[cellIndex];

                if (!aabb.overlapTest(cell.AABox)) {
                    continue;
                }

                if (cell.isLeaf()) {
                    let nTris = cell.triangleIndices.length;
                    for (var i = 0; i < nTris; i++) {
                        let triangle = this.getTriangle(cell.triangleIndices[i]);
                        if (triangle.counter != this._testCounter) {
                            triangle.counter = this._testCounter;
                            if (aabb.overlapTest(triangle.get_boundingBox())) {
                                triangles.push(cell.triangleIndices[i]);
                            }
                        }
                    }
                } else {
                    for (i = 0; i < OctreeCell.NUM_CHILDREN; i++) {
                        this._cellsToTest.push(cell.childCellIndices[i]);
                    }
                }
            }
            return triangles.length;

        }

        dumpStats() {

            var maxTris = 0, numTris, cellIndex, cell;

            var cellsToProcess = [];
            cellsToProcess.push(0);

            while (cellsToProcess.length != 0) {
                cellIndex = cellsToProcess.pop();

                cell = cell[cellIndex];
                if (cell.isLeaf()) {

                    numTris = cell.triangleIndices.length;
                    if (numTris > maxTris) {
                        maxTris = numTris;
                    }
                } else {
                    for (var i = 0; i < OctreeCell.NUM_CHILDREN; i++) {
                        if ((cell.childCellIndices[i] >= 0) && (cell.childCellIndices[i] < this._cells.length)) {
                            cellsToProcess.push(cell.childCellIndices[i]);
                        }
                    }
                }
            }

        }

        createAABox(aabb: JAABox, _id) {

            var dims = JNumber3D.getScaleVector(aabb.maxPos.subtract(aabb.minPos), 0.5);
            switch (_id) {
                case 0:
                    var offset = new Vector3D(1, 1, 1);
                    break;
                case 1:
                    var offset = new Vector3D(1, 1, 0);
                    break;
                case 2:
                    var offset = new Vector3D(1, 0, 1);
                    break;
                case 3:
                    var offset = new Vector3D(1, 0, 0);
                    break;
                case 4:
                    var offset = new Vector3D(0, 1, 1);
                    break;
                case 5:
                    var offset = new Vector3D(0, 1, 0);
                    break;
                case 6:
                    var offset = new Vector3D(0, 0, 1);
                    break;
                case 7:
                    var offset = new Vector3D(0, 0, 0);
                    break;
                default:
                    var offset = new Vector3D(0, 0, 0);
                    break;
            }

            var result = new JAABox();
            result.minPos = aabb.minPos.add(new Vector3D(offset.x * dims.x, offset.y * dims.y, offset.z * dims.z));
            result.maxPos = result.minPos.add(dims);

            dims.scaleBy(0.00001);
            result.minPos = result.minPos.subtract(dims);
            result.maxPos = result.maxPos.add(dims);

            return result;

        }

        doesTriangleIntersectCell(triangle, cell) {
            return triangle.get_boundingBox().overlapTest(cell.AABox);
            /*
            if (!triangle.get_boundingBox().overlapTest(cell.AABox)) {
                return false;
            }
            if (cell.AABox.isPointInside(this.getVertex(triangle.getVertexIndex(0))) ||
                cell.AABox.isPointInside(this.getVertex(triangle.getVertexIndex(1))) ||
                cell.AABox.isPointInside(this.getVertex(triangle.getVertexIndex(2)))) {
                    return true;
                }
			
            var tri = new JTriangle(this.getVertex(triangle.getVertexIndex(0)), this.getVertex(triangle.getVertexIndex(1)), this.getVertex(triangle.getVertexIndex(2)));
            var edge;
            var seg;
            var edges = cell.get_egdes();
            var pts = cell.get_points();
            for (var i = 0; i < 12; i++ ) {
                edge = edges[i];
                seg = new JSegment(pts[edge.ind0], pts[edge.ind1].subtract(pts[edge.ind0]));
                if (tri.segmentTriangleIntersection(null, seg)) {
                    return true;
                }
            }
		
            var pt0;
            var pt1;
            for (i = 0; i < 3; i++ ) {
                pt0 = tri.getVertex(i);
                pt1 = tri.getVertex((i + 1) % 3);
                if (cell.AABox.segmentAABoxOverlap(new JSegment(pt0, pt1.subtract(pt0)))) {
                    return true;
                }
            }
            return false;
            */

        }

        incrementTestCounter() {

            ++this._testCounter;
            if (this._testCounter == 0) {
                let numTriangles = this._triangles.length;
                for (var i = 0; i < numTriangles; i++) {
                    this._triangles[i].counter = 0;
                }
                this._testCounter = 1;
            }

        }

    }
}