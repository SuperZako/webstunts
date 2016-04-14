
/// <reference path="JOctree.ts"/>

module jiglib {

    export class JTriangleMesh extends RigidBody {
        _octree: JOctree = null; // JOctree
        _maxTrianglesPerCell: number = null; // int
        _minCellSize: number = null; // Number
        _skinVertices: Vector3D[] = null; // Vector3D

        constructor(skin, initPosition, initOrientation, maxTrianglesPerCell, minCellSize) {
            super(skin);

            this.get_currentState().position = initPosition.clone();
            this.get_currentState().orientation = initOrientation.clone();
            this._maxTrianglesPerCell = maxTrianglesPerCell;
            this._minCellSize = minCellSize;

            this.set_movable(false);

            if (skin) {
                this._skinVertices = skin.vertices;
                this.createMesh(this._skinVertices, skin.indices);

                this._boundingBox = this._octree.boundingBox().clone();
                skin.transform = JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getTranslationMatrix(this.get_currentState().position.x, this.get_currentState().position.y, this.get_currentState().position.z));
            }

            this._type = "TRIANGLEMESH";

        }

        createMesh(vertices, triangleVertexIndices) {

            var vts = [];

            var transform = JMatrix3D.getTranslationMatrix(this.get_currentState().position.x, this.get_currentState().position.y, this.get_currentState().position.z);
            transform = JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, transform);

            var i = 0;
            //for (var vertices_i = 0, vertices_l = vertices.length, _point; (vertices_i < vertices_l) && (_point = vertices[vertices_i]); vertices_i++) {
            for (let _point of vertices) {
                vts[i++] = transform.transformVector(_point);
            }

            this._octree = new JOctree();

            this._octree.addTriangles(vts, vts.length, triangleVertexIndices, triangleVertexIndices.length);
            this._octree.buildOctree(this._maxTrianglesPerCell, this._minCellSize);


        }

        get_octree() {

            return this._octree;

        }
        segmentIntersect(out, seg, state) {

            var segBox = new JAABox();
            segBox.addSegment(seg);

            var potentialTriangles = [];
            var numTriangles = this._octree.getTrianglesIntersectingtAABox(potentialTriangles, segBox);

            var bestFrac = JMath3D.NUM_HUGE;
            var tri;
            var meshTriangle;
            for (var iTriangle = 0; iTriangle < numTriangles; iTriangle++) {
                meshTriangle = this._octree.getTriangle(potentialTriangles[iTriangle]);

                tri = new JTriangle(this._octree.getVertex(meshTriangle.getVertexIndex(0)), this._octree.getVertex(meshTriangle.getVertexIndex(1)), this._octree.getVertex(meshTriangle.getVertexIndex(2)));

                if (tri.segmentTriangleIntersection(out, seg)) {
                    if (out.frac < bestFrac) {
                        bestFrac = out.frac;
                        out.position = seg.getPoint(bestFrac);
                        out.normal = meshTriangle.get_plane().get_normal();
                    }
                }
            }
            out.frac = bestFrac;
            return bestFrac < JMath3D.NUM_HUGE;

        }

        updateState() {
            super.updateState();
            var vts = [];

            var transform = JMatrix3D.getTranslationMatrix(this.get_currentState().position.x, this.get_currentState().position.y, this.get_currentState().position.z);
            transform = JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, transform);

            var i = 0;
            for (var _skinVertices_i = 0, _skinVertices_l = this._skinVertices.length, _point; (_skinVertices_i < _skinVertices_l) && (_point = this._skinVertices[_skinVertices_i]); _skinVertices_i++) {
                vts[i++] = transform.transformVector(_point);
            }

            this._octree.updateTriangles(vts);
            this._octree.buildOctree(this._maxTrianglesPerCell, this._minCellSize);

            this._boundingBox = this._octree.boundingBox().clone();

        }

        getInertiaProperties(m) {
            return new Matrix3D();
        }

        updateBoundingBox() {
        }

    }
}