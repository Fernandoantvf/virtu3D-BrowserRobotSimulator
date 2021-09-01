/* 
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

function RevoluteDeltaKineConverter(robot, WP, WB, L, l) {
    this.WBWP = WP - WB;
    this.q = [0, 0, 0];
    this.ep = [0, 0, 0];
    //Math.js usefull sitax:
    //math.dot(A,B)
    //math.cross(A,B)
    //Declaring of a vector for matrix manipulation:
    // var position = math.matrix([0, 0, 0]);
    this.ForwardConversion = function () {
        this.q = [robot.legs[0].angle, robot.legs[1].angle, robot.legs[2].angle];
        var sq = [Math.sin(this.q[0]), Math.sin(this.q[1]), Math.sin(this.q[2])];
        var cq = [Math.cos(this.q[0]), Math.cos(this.q[1]), Math.cos(this.q[2])];

        var Av0 = [0, 0, 0];
        var Av1 = [0, 0, 0];
        var Av2 = [0, 0, 0];

        Av0[0] = 0;
        Av0[1] = -L * cq[0] - WB + WP;
        Av0[2] = -L * sq[0];

        Av1[0] = Math.sqrt(3) * (L * cq[1] + WB - WP) / 2;
        Av1[1] = 0.5 * L * cq[1] + 0.5 * WB - 0.5 * WP;
        Av1[2] = -L * sq[1];

        Av2[0] = -Math.sqrt(3) * (L * cq[2] + WB - WP) / 2;
        Av2[1] = 0.5 * L * cq[2] + 0.5 * WB - 0.5 * WP;
        Av2[2] = -L * sq[2];

        var ep0, ep1;
        var c0, c1, c2;

        c0 = new THREE.Vector3(Av0[0], Av0[1], Av0[2]);
        c1 = new THREE.Vector3(Av1[0], Av1[1], Av1[2]);
        c2 = new THREE.Vector3(Av2[0], Av2[1], Av2[2]);

        /*
         //Attept of calculating intersections by projecting from virtual knee triangle
         var virtKneeDeltaCent = c0.clone();
         virtKneeDeltaCent.add(c1);
         virtKneeDeltaCent.add(c2);
         virtKneeDeltaCent.divideScalar(3);
         var intersecDistPlane = virtKneeDeltaCent.distanceTo(c0);
         var c0DeltCent = virtKneeDeltaCent.clone();
         c0DeltCent.sub(c0);
         var interseDistPerpendicular = Math.sqrt(Math.pow(l,2)- Math.pow(intersecDistPlane,2));
         var deltPlaneNormal = virtKneeDeltaCent.clone();
         deltPlaneNormal.sub(c1);
         deltPlaneNormal.cross(c0DeltCent);
         deltPlaneNormal.normalize();
         var ep0 = deltPlaneNormal.clone();
         ep0.multiplyScalar(interseDistPerpendicular);
         var ep1 = ep0.clone();
         ep1.multiplyScalar(-1);
         ep0.add(virtKneeDeltaCent);
         ep1.add(virtKneeDeltaCent);
         */


        //Intersect 2 spheres of equal radius:
        //http://mathworld.wolfram.com/Sphere-SphereIntersection.html
        var c0c1 = c1.clone();
        c0c1.sub(c0);
        var intersecCircleNormal = c0c1.clone();
        intersecCircleNormal.divideScalar(2);
        var intersecCircleCenter = c0.clone();
        intersecCircleCenter.add(intersecCircleNormal);
        var centerDist = c0.distanceTo(c1);
        intersecCircleNormal.divideScalar(centerDist/2);
        
        

        //var intersecCircleRadius = 1/(2*centerDist)* Math.sqrt(4*Math.pow(centerDist,2)*Math.pow(l,2)- Math.pow(centerDist,4));
        //var intersecCircleRadius = 1/(2*centerDist) * Math.sqrt(4*Math.pow(centerDist,2)*Math.pow(l,2) - Math.pow((Math.pow(centerDist,2) - Math.pow(l,2) + Math.pow(l,2)),2));
        var intersecCircleRadius = Math.sqrt(Math.pow(l, 2) - Math.pow(centerDist / 2, 2));

        var intersecCirclePlane = new THREE.Plane(intersecCircleNormal);

        //Intersect sphere with plane of circle
        //http://www.ambrsoft.com/TrigoCalc/Sphere/SpherePlaneIntersection_.htm
        var distanceSpherePlane = Math.abs(intersecCirclePlane.distanceToPoint(intersecCircleCenter) - intersecCirclePlane.distanceToPoint(c2));
        
        
        var intersecCircleCenter2 = intersecCircleNormal.clone();
        intersecCircleCenter2.multiplyScalar(distanceSpherePlane);
        
        intersecCircleCenter2.add(c2);

        var intersecCircleRadius2 = Math.sqrt(Math.pow(l, 2) - Math.pow(distanceSpherePlane, 2));

        //Intersect 2 circles
        var icc0icc1 = intersecCircleCenter2.clone();
        icc0icc1.sub(intersecCircleCenter);
        var centerDist = intersecCircleCenter.distanceTo(intersecCircleCenter2);
        var intersecDistOnAxis = (Math.pow(centerDist, 2) - Math.pow(intersecCircleRadius2, 2) + Math.pow(intersecCircleRadius, 2)) / (2 * centerDist);

        var intersecProjOnaxis = icc0icc1.clone();
        intersecProjOnaxis.normalize();
        intersecProjOnaxis.multiplyScalar(intersecDistOnAxis);
        intersecProjOnaxis.add(intersecCircleCenter);

        var intersecDistPrependicular = Math.sqrt(Math.pow(intersecCircleRadius, 2) - Math.pow(intersecDistOnAxis, 2));

        var ep0 = icc0icc1.clone();
        ep0.cross(intersecCircleNormal);
        ep0.normalize();
        ep0.multiplyScalar(intersecDistPrependicular);
        var ep1 = ep0.clone();
        ep1.multiplyScalar(-1);

        ep0.add(intersecProjOnaxis);
        ep1.add(intersecProjOnaxis);


        //console.log("icc0icc1=(" + icc0icc1.x.toString() + "," + icc0icc1.Y.toString() + "," + icc0icc1.Z.toString() + ")");
        //console.log("c0c1=(" + c0c1.x.toString() + "," + c0c1.Y.toString() + "," + c0c1.Z.toString() + ")");

        this.ep[0] = ep0.x;
        this.ep[1] = ep0.y;
        this.ep[2] = ep0.z;

        //or
        
        //this.ep[0] = ep1.x;
        //this.ep[1] = ep1.y;
        //this.ep[2] = ep1.z;

        this.ep[0] = this.ep[0] + robot.position.x;
        this.ep[1] = this.ep[1] + robot.position.y;
        this.ep[2] = this.ep[2] + robot.position.z;
        
        robot.platform.group.position.set(this.ep[0], this.ep[1], this.ep[2]);
        robot.PositionTieRods();
    };

    this.ReverseConversion = function (robot) {
        var ep0 = new THREE.Vector3(robot.platform.group.position.x, robot.platform.group.position.y, robot.platform.group.position.z);

        //circle center positions are leg position
        //legs[n].group.position;
        //Circle radiuses are l
        //circle plane normals are:
        //legs[n].rotAxisDir
        //zeroDirectionVectors are:
        //legs[n].zeroDirection
        var virtualHips = [];


        for (var i = 0; i < 3; i++) {
            var virtualHip = robot.legs[i].zeroDirection.clone();
            virtualHip.normalize();
            virtualHip.multiplyScalar(this.WBWP);
            virtualHip.add(robot.legs[i].group.position);
            virtualHips.push(virtualHip);


            //Intersect sphere with plane of circle
            //http://www.ambrsoft.com/TrigoCalc/Sphere/SpherePlaneIntersection_.htm

            var intersecCirclePlane = new THREE.Plane(robot.legs[i].rotAxisDir);

            var distanceSpherePlane = Math.abs(intersecCirclePlane.distanceToPoint(ep0) - intersecCirclePlane.distanceToPoint(virtualHip));
            var intersecCircleCenter2 = intersecCirclePlane.projectPoint(ep0);
            intersecCircleCenter2.sub(ep0);
            intersecCircleCenter2.normalize();
            intersecCircleCenter2.multiplyScalar(distanceSpherePlane);
            intersecCircleCenter2.add(ep0);

            var intersecCircleRadius2 = Math.sqrt(Math.pow(l, 2) - Math.pow(distanceSpherePlane, 2));


            //Intersect 2 circles
            var icc0icc1 = intersecCircleCenter2.clone();
            icc0icc1.sub(virtualHip);
            var centerDist = virtualHip.distanceTo(intersecCircleCenter2);
            var intersecDistOnAxis = (Math.pow(centerDist, 2) - Math.pow(intersecCircleRadius2, 2) + Math.pow(L, 2)) / (2 * centerDist);

            var intersecProjOnaxis = icc0icc1.clone();
            intersecProjOnaxis.normalize();
            intersecProjOnaxis.multiplyScalar(intersecDistOnAxis);
            intersecProjOnaxis.add(virtualHip);

            var intersecDistPrependicular = Math.sqrt(Math.pow(L, 2) - Math.pow(intersecDistOnAxis, 2));

            var kp0 = icc0icc1.clone();
            kp0.cross(robot.legs[i].rotAxisDir);
            kp0.normalize();
            kp0.multiplyScalar(intersecDistPrependicular);
            var kp1 = kp0.clone();
            kp1.multiplyScalar(-1);

            kp0.add(intersecProjOnaxis);
            kp1.add(intersecProjOnaxis);

            
            var angleVec = kp0.clone();
            angleVec.sub(virtualHip);
            var angle = angleVec.angleTo(robot.legs[i].zeroDirection);
            if (angleVec.z > 0 )
                angle = - angle;
            this.q[i] = angle;
            robot.legs[i].setAngle(this.q[i]);
        }
        robot.PositionTieRods();
    };


}
