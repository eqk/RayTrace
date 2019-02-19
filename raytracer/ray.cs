using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;

namespace RayTracer
{
    public class Ray
    {
        public static readonly float EPSILON = 0.0001f;

        public Point3D start, dir;

        public Ray(Point3D st, Point3D end)
        {
            start = new Point3D(st);
            dir = Point3D.Norm(end - st);
        }

        private Ray()
        {
        }

        public Point3D TPos(float t)
        {
            return start + dir * t;
        }

        public static Ray BuildRay(Point3D st, Point3D dir)
        {
            return new Ray {start = new Point3D(st), dir = new Point3D(dir)};
        }


        public bool IntersectTriangle(Point3D vertex0, Point3D vertex1, Point3D vertex2, out float t) // TODO 
        {
            t = -1;
            var edge1 = vertex1 - vertex0;
            var edge2 = vertex2 - vertex0;
            var h = dir * edge2;
            var a = Point3D.Scalar(edge1, h);
            
            if (a > -EPSILON && a < EPSILON)
                return false; // This ray is parallel to this triangle.
            
            var f = 1.0f / a;
            var s = start - vertex0;
            var u = Point3D.Scalar(s, h) * f;
            if (u < 0.0 || u > 1.0)
                return false;
            var q = s * edge1;
            var v = Point3D.Scalar(dir, q) * f;
            if (v < 0.0 || u + v > 1.0)
                return false;
            
            // At this stage we can compute t to find out where the intersection point is on the line.
            t = Point3D.Scalar(edge2, q) * f;
            
            return t > EPSILON;
        }
    }


    public class Hit
    {
        public readonly float hit_point;
        private readonly Ray _castedRay;
        public Point3D normal;
        public Material mat;
        public readonly bool success;

        public Hit(float hp, Ray r, Point3D n, Material m)
        {
            hit_point = hp;
            _castedRay = r;
            normal = new Point3D(n);
            mat = m;
            success = true;
        }


        public Hit()
        {
            success = false;
            hit_point = -1;
            _castedRay = null;
            normal = null;
        }

        public Point3D Shade(Light l, Point3D hp, Point3D eye)
        {
            //  Point3D d = Point3D.norm(hp - l.position);
            //  d = l.clr * Math.Max(Point3D.scalar(d, normal),0) ;

            var l2 = Point3D.Norm(l.position - hp);
            var v2 = Point3D.Norm(eye - hp);
            var r = ReflectVec(l2 * -1, normal);
            var diff = mat.dif_coef * l.clr * Math.Max(Point3D.Scalar(normal, l2), 0.0f);
            var spec = mat.spec_coef * l.clr *
                           (float) Math.Pow(Math.Max(Point3D.Scalar(r, v2), 0.0f), mat.shine_coef);


            return Point3D.blend(diff, mat.clr);
        }

        public Ray Reflect(Point3D hp)
        {
            return Ray.BuildRay(hp, Point3D.Norm(_castedRay.dir - 2 * normal * Point3D.Scalar(normal, _castedRay.dir)));
        }

        private static Point3D ReflectVec(Point3D v, Point3D n)
        {
            return Point3D.Norm(v - 2 * n * Point3D.Scalar(n, v));
        }

        public Ray Refract(Point3D hp, float eta)
        {
            var nidot = Point3D.Scalar(normal, _castedRay.dir);
            var k = 1.0f - eta * eta * (1.0f - nidot * nidot);
            if (k >= 0)
            {
                k = (float) Math.Sqrt(k);
                return Ray.BuildRay(hp, Point3D.Norm(eta * _castedRay.dir - (k + eta * nidot) * normal));
            }
            else
                return null;
        }
    }

    public struct Material
    {
        public Point3D clr;
        public float reflection_coef;
        public float refraction_coef;
        public float env_coef;

        public float amb_coef;
        public float dif_coef;
        public float spec_coef;
        public float shine_coef;

        public Material(Point3D c, float refl, float refr, float ec, float ac, float dc, float sc, float shc)
        {
            clr = new Point3D(c);
            reflection_coef = refl;
            refraction_coef = refr;
            env_coef = ec;
            amb_coef = ac;
            dif_coef = dc;
            spec_coef = sc;
            shine_coef = shc;
        }

        public void SetColor(Color c)
        {
            clr = new Point3D(c.R / 255.0f, c.G / 255.0f, c.B / 255.0f);
        }
    }
}