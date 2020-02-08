package com.spartronics4915.lib.math.threedim;

import java.lang.Math;
import java.util.Random;
import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrix3x3;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class TransformationsTest
{
    double kEpsilon = 1e-9;

    @Test
    void testVec3()
    {
        Vec3 aPt = new Vec3(1,2,3);
        Vec3 bPt = new Vec3(2,4,6);

        double aLen = aPt.length();
        assertEquals(aLen, Math.sqrt(1+4+9), kEpsilon);
        Vec3 nPt = new Vec3(aPt);
        nPt.normalize();
        assertEquals(nPt.length(), 1, kEpsilon);

        DMatrix3x3 op = aPt.outerProduct(bPt);
        op.print(); // can be viewed if you "Run Test" in vscode, also in log
        assertEquals(op.a11, 2, kEpsilon);
        assertEquals(op.a22, 8, kEpsilon);
        assertEquals(op.a33, 18, kEpsilon);

        Vec3 cross = new Vec3(1,0,0).cross(new Vec3(0,1,0));
        assert(cross.equals(new Vec3(0,0,1), kEpsilon));

        Vec3 delta = new Vec3(1,1,1);
        Vec3 sub = cross.subtract(delta);
        Vec3 add = sub.add(delta);
        assert(add.equals(cross, kEpsilon));
    }

    @Test
    void testAffine3()
    {
        Affine3 m = new Affine3();
        Vec3 aPt = new Vec3(1,2,3);

        // test for identity
        Vec3 v1 = m.transformPoint(aPt);
        assertEquals(v1.a1, aPt.a1);
        assertEquals(v1.a2, aPt.a2);
        assertEquals(v1.a3, aPt.a3);

        // test translation
        m = Affine3.fromTranslation(1,2,3);
        Vec3 v = m.getTranslation();
        assertEquals(v.a1, 1, kEpsilon);
        assertEquals(v.a2, 2, kEpsilon);
        assertEquals(v.a3, 3, kEpsilon);
        // m.print();

        Vec3 v2 = m.transformPoint(new Vec3(1,2,3));
        assertEquals(v2.a1, 2, kEpsilon);
        assertEquals(v2.a2, 4, kEpsilon);
        assertEquals(v2.a3, 6, kEpsilon);

        Vec3 xAxis = new Vec3(1,0,0);
        Vec3 yAxis = new Vec3(0,1,0);
        Vec3 zAxis = new Vec3(0,0,1);
        Vec3 origin = new Vec3(0,0,0);

        // simple rotation
        Affine3 R1 = Affine3.fromRotation(90, zAxis);
        R1.print();
        Vec3 p1 = R1.transformVector(xAxis);
        p1.print();
        assert(p1.equals(yAxis, kEpsilon));

        // rotation around the point [1,0,0]
        Affine3 R2 = Affine3.fromRotation(90, zAxis, new Vec3(1,0,0));
        R2.print();
        Vec3 p2 = R2.transformPoint(origin);
        p2.print();
        assert(p2.equals(new Vec3(1, -1, 0), kEpsilon));
    }

    @Test
    void testAffine3Rot()
    {
        Random rgen = new Random();
        double randAngle = 360 * (rgen.nextDouble()-.5); // [-180, 180]
        Vec3 randDir = new Vec3(rgen.nextDouble()-.5, rgen.nextDouble()-.5, 
                                rgen.nextDouble()-.5);
        randDir.normalize();
        Vec3 randPt = new Vec3(rgen.nextDouble()-.5, rgen.nextDouble()-.5, 
                                rgen.nextDouble()-.5);
        Affine3 R0 = Affine3.fromRotation(randAngle, randDir, randPt);
        Affine3 R0a = Affine3.fromRotation(randAngle-360, randDir, randPt); 
        assert(R0.equals(R0a, kEpsilon));

        Affine3 R0b = Affine3.fromRotation(randAngle, randDir, randPt);
        Affine3 R0c = Affine3.fromRotation(-randAngle, randDir.getOpposite(), randPt);
        assert(R0b.equals(R0c, kEpsilon));

        Affine3 I = new Affine3();
        Affine3 R0d = Affine3.fromRotation(360, randDir);
        assert(I.equals(R0d, kEpsilon));

        randDir = new Vec3(.32281269, -.08711045, -.46931146);
        randPt = new Vec3(-0.49508149, -0.10203337,  0.41560636);
        Affine3 R0e = Affine3.fromRotation(90, randDir.asUnit(), randPt);
        double t = R0e.trace();
        assert(Math.abs(2 - t) < kEpsilon);
    }

    @Test
    void testApplication()
    {


    }

}