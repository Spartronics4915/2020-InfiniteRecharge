package com.spartronics4915.lib.math.threedim;
import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrix4;
import org.ejml.dense.fixed.CommonOps_DDF4;
/* http://ejml.org/javadoc/org/ejml/dense/fixed/CommonOps_DDF4.html */

/**
 *  Robust+compact representation of 3D rotation
 *
 *  Quaternions w+ix+jy+kz are represented as [w, x, y, z].
 *  see: 
 *      ttps://eater.net/quaternions/intro
 *      https://www.youtube.com/watch?v=jlskQDR8-bY  (Mathoma)
 *      https://www.youtube.com/watch?v=d5EgbgTm0Bg (3Blue1Brown)
 *      https://www.youtube.com/watch?v=zjMuIxRvygQ (3Blue1Brown)
 */

class Quaternion
{
    static final double kEpsilon = 1e-9;

    /* factory static methods -----------------------------------------------*/

    /* methods -----------------------------------------------*/
    private DMatrix4 mQ;

    /**
     * 4 double contructor - each of the 4 quaternion components
     * @param q0
     * @param q1
     * @param q2
     * @param q3
     */
    public Quaternion(double q0, double q1, double q2, double q3)
    {
        this.mQ = new DMatrix4();
        this.mQ.a1 = q0;
        this.mQ.a2 = q1;
        this.mQ.a3 = q2;
        this.mQ.a4 = q3;
    }

    /**
     * default contructor, produces identity
     */
    public Quaternion()
    {
        this(1, 0, 0, 0);
    }

    /**
     * angle/axis contructor
     * @param angle - measured in degrees
     * @param axis
     */
    public Quaternion(double angle, Vec3 axis)
    {
        this.mQ = new DMatrix4();
        this.mQ.a1 = 0.0; 
        this.mQ.a2 = axis.a1;
        this.mQ.a3 = axis.a2;
        this.mQ.a4 = axis.a3;
        double len = axis.length();
        double rads = Math.toRadians(angle);
        if(len > kEpsilon)
        {
            double s = Math.sin(rads/2.0) / len;
            CommonOps_DDF4.scale(s, this.mQ);
        }
        this.mQ.a1 = Math.cos(rads/2.0);
    }

    /**
     * euler-angle constructor
     * @param ai
     * @param aj
     * @param ak
     * @param axes
     */
    public Quaternion(double ai, double aj, double ak, String axes)
    {
    }

    public Quaternion(DMatrix4x4 m)
    {
    }

    DMatrix4 asDMatrix4()
    {
        return this.mQ;
    }

    /**
     * converts the current quaternion values into 4x4 matrix form.
     * nb: translation and projection fields are 0.
     * @return
     */
    DMatrix4x4 asDMatrix4x4()
    {
        DMatrix4x4 result = new DMatrix4x4();
        DMatrix4 q = this.mQ;
        double dot = CommonOps_DDF4.dot(q, q);
        if(dot < kEpsilon)
            CommonOps_DDF4.setIdentity(result);
        else
        {
            /* outer product(q,q) */
            DMatrix4x4 qq = new DMatrix4x4();
            qq.a11 = q.a1 * q.a1;
            qq.a12 = q.a2 * q.a1;
            qq.a13 = q.a3 * q.a1;
            qq.a14 = q.a4 * q.a1;
            qq.a21 = q.a1 * q.a2;
            qq.a22 = q.a2 * q.a2;
            qq.a23 = q.a3 * q.a2;
            qq.a24 = q.a4 * q.a2;
            qq.a31 = q.a1 * q.a3;
            qq.a32 = q.a2 * q.a3;
            qq.a33 = q.a3 * q.a3;
            qq.a34 = q.a4 * q.a3;
            qq.a41 = q.a1 * q.a4;
            qq.a42 = q.a2 * q.a4;
            qq.a43 = q.a3 * q.a4;
            qq.a44 = q.a4 * q.a4;

            result.a11 = 1. - qq.a33 - qq.a44;
            result.a12 = qq.a23 - qq.a41;
            result.a13 = qq.a24 + qq.a31;
            result.a14 = 0;

            result.a21 = qq.a23 + qq.a41;
            result.a22 = 1. - qq.a22 - qq.a44;
            result.a23 = qq.a34 - qq.a21;
            result.a24 = 0.;

            result.a31 = qq.a24 - qq.a31;
            result.a32 = qq.a34 + qq.a21;
            result.a33 = 1. - qq.a22 - qq.a33;
            result.a34 = 0.;

            result.a41 = 0;
            result.a42 = 0;
            result.a43 = 0;
            result.a44 = 1;
        }
        return result;
    }

    boolean equals(Quaternion rhs)
    {
        return this.equals(rhs, 1e-9);
    }

    boolean equals(Quaternion rhs, double epsilon)
    {
        if(Math.abs(this.mQ.a1-rhs.mQ.a1) > epsilon) return false;
        if(Math.abs(this.mQ.a2-rhs.mQ.a2) > epsilon) return false;
        if(Math.abs(this.mQ.a3-rhs.mQ.a3) > epsilon) return false;
        if(Math.abs(this.mQ.a4-rhs.mQ.a4) > epsilon) return false;
        return true;
    }


}