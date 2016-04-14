/// <reference path="../cof/JConfig.ts"/>
/// <reference path="../data/PlaneData.ts"/>
/// <reference path="../geom/Vector3D.ts"/>
/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>

module jiglib {


    export class JIndexedTriangle {
        counter = null; // int
        private _vertexIndices = null; // uint
        private _plane = null; // PlaneData
        private _boundingBox = null; // JAABox

        constructor() {

            this.counter = 0;
            this._vertexIndices = [];
            this._vertexIndices[0] = -1;
            this._vertexIndices[1] = -1;
            this._vertexIndices[2] = -1;
            this._plane = new PlaneData();
            this._boundingBox = new JAABox();

        }

        setVertexIndices(i0, i1, i2, vertexArray) {

            this._vertexIndices[0] = i0;
            this._vertexIndices[1] = i1;
            this._vertexIndices[2] = i2;

            this._plane.setWithPoint(vertexArray[i0], vertexArray[i1], vertexArray[i2]);

            this._boundingBox.clear();
            this._boundingBox.addPoint(vertexArray[i0]);
            this._boundingBox.addPoint(vertexArray[i1]);
            this._boundingBox.addPoint(vertexArray[i2]);

        }

        updateVertexIndices(vertexArray) {

            var i0, i1, i2;
            i0 = this._vertexIndices[0];
            i1 = this._vertexIndices[1];
            i2 = this._vertexIndices[2];

            this._plane.setWithPoint(vertexArray[i0], vertexArray[i1], vertexArray[i2]);

            this._boundingBox.clear();
            this._boundingBox.addPoint(vertexArray[i0]);
            this._boundingBox.addPoint(vertexArray[i1]);
            this._boundingBox.addPoint(vertexArray[i2]);

        }

        get_vertexIndices() {

            return this._vertexIndices;

        }

        getVertexIndex(iCorner) {

            return this._vertexIndices[iCorner];

        }

        get_plane() {

            return this._plane;

        }

        get_boundingBox() {

            return this._boundingBox;

        }

    }
}