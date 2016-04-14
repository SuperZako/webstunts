
/// <reference path="../cof/JConfig.ts"/>

/// <reference path="../data/SpanData.ts"/>

/// <reference path="../geom/Vector3D.ts"/>

/// <reference path="../math/JMath3D.ts"/>
/// <reference path="../math/JNumber3D.ts"/>

/// <reference path="../physics/MaterialProperties.ts"/>
/// <reference path="../physics/RigidBody.ts"/>

/// <reference path="CollDetectFunctor.ts"/>
/// <reference path="CollPointInfo.ts"/>
/// <reference path="CollDetectInfo.ts"/>



module jiglib {
    export class CollisionInfo {
        mat = new MaterialProperties(); // MaterialProperties
        objInfo: CollDetectInfo= null; // CollDetectInfo
        dirToBody: Vector3D= null; // Vector3D
        pointInfo: CollPointInfo[]= null; // CollPointInfo
        satisfied:boolean = null; // Boolean
    }
}
