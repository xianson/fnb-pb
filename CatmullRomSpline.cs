using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        // Centripetal Catmull-Rom spline (alpha = 0.5). Ported verbatim from the mod's
        // Routing/CatmullRomSpline.cs; the display-only Injected/InjectedRadius arrays are dropped
        // (a PB draws no route ribbon), which is the only difference.
        public sealed class CatmullRomSpline
        {
            public readonly Vector3D[] ControlPoints;
            public int SegmentCount { get { return Math.Max(0, ControlPoints.Length - 1); } }

            // Minimum gap (m) between consecutive control points.
            public const double MinCpGap = 1e-2;

            // Floor on a centripetal knot interval (|dP|^0.5).
            const double KnotEps = 1e-4;

            public CatmullRomSpline(Vector3D[] points)
            {
                if (points == null) points = new Vector3D[0];
                // Dedupe consecutive near-duplicates (degenerate segments).
                var deduped = new System.Collections.Generic.List<Vector3D>(points.Length);
                for (int i = 0; i < points.Length; i++)
                {
                    if (deduped.Count > 0
                        && (points[i] - deduped[deduped.Count - 1]).LengthSquared() < MinCpGap * MinCpGap)
                        continue;
                    deduped.Add(points[i]);
                }
                if (deduped.Count < 2)
                    throw new ArgumentException("spline needs 2+ distinct control points");
                ControlPoints = deduped.ToArray();
            }

            // Four control points and three local knot intervals for the segment from ControlPoints[seg].
            void SegmentKnots(int seg, out Vector3D p0, out Vector3D p1, out Vector3D p2, out Vector3D p3,
                out double t0, out double t1, out double t2, out double t3)
            {
                p0 = ControlPoints[Math.Max(0, seg - 1)];
                p1 = ControlPoints[seg];
                p2 = ControlPoints[seg + 1];
                p3 = ControlPoints[Math.Min(ControlPoints.Length - 1, seg + 2)];
                t0 = 0.0;
                t1 = t0 + KnotInterval(p0, p1);
                t2 = t1 + KnotInterval(p1, p2);
                t3 = t2 + KnotInterval(p2, p3);
            }

            // KNOT CACHE. SegmentKnots is a pure function of seg but costs three Math.Pow, and every
            // caller evaluates many u inside one segment (48 bake samples, 8 ClosestU iterations).
            // Caching by seg is bit-identical -- same inputs, same doubles -- and removes ~97% of the
            // Math.Pow calls in the bake.
            int _kcSeg = -1;
            Vector3D _kcP0, _kcP1, _kcP2, _kcP3;
            double _kcT0, _kcT1, _kcT2, _kcT3;

            void CachedKnots(int seg)
            {
                if (_kcSeg == seg) return;
                SegmentKnots(seg, out _kcP0, out _kcP1, out _kcP2, out _kcP3,
                             out _kcT0, out _kcT1, out _kcT2, out _kcT3);
                _kcSeg = seg;
            }

            static double KnotInterval(Vector3D a, Vector3D b)
            {
                // alpha = 0.5 -> interval = |b - a|^0.5 = (|b - a|^2)^0.25.
                double d2 = (b - a).LengthSquared();
                double dt = Math.Pow(d2, 0.25);
                return dt < KnotEps ? KnotEps : dt;
            }

            // Evaluate spline at u in [0, SegmentCount].
            public Vector3D Position(double u, out Vector3D tangent)
            {
                if (u < 0.0) u = 0.0;
                if (u > SegmentCount) u = SegmentCount;
                int seg = (int)Math.Floor(u);
                if (seg >= SegmentCount) seg = SegmentCount - 1;
                double t = u - seg;
                Vector3D d2;
                return EvalSegment(seg, t, out tangent, out d2);
            }

            // Sample position, tangent and curvature at u with ONE evaluation. Position and
            // CurvatureAt derive (seg, t) by identical code and both call EvalSegment, so calling
            // them in sequence evaluated the same segment twice; this returns all three.
            public Vector3D Sample(double u, out Vector3D tangent, out double kappa)
            {
                if (u < 0.0) u = 0.0;
                if (u > SegmentCount) u = SegmentCount;
                int seg = (int)Math.Floor(u);
                if (seg >= SegmentCount) seg = SegmentCount - 1;
                double t = u - seg;
                Vector3D secondDeriv;
                Vector3D p = EvalSegment(seg, t, out tangent, out secondDeriv);
                double fdMag2 = tangent.LengthSquared();
                kappa = fdMag2 < 1e-12
                    ? 0.0
                    : Vector3D.Cross(tangent, secondDeriv).Length() / Math.Pow(fdMag2, 1.5);
                return p;
            }

            // Barry-Goldman recursive evaluation plus first and second derivatives.
            Vector3D EvalSegment(int seg, double t, out Vector3D firstDeriv, out Vector3D secondDeriv)
            {
                CachedKnots(seg);
                double t0 = _kcT0, t1 = _kcT1, t2 = _kcT2, t3 = _kcT3;
                Vector3D p0 = _kcP0, p1 = _kcP1, p2 = _kcP2, p3 = _kcP3;

                double s = t1 + t * (t2 - t1);

                Vector3D a1, a2, a3, da1, da2, da3;
                Lerp(p0, p1, t0, t1, s, out a1, out da1);
                Lerp(p1, p2, t1, t2, s, out a2, out da2);
                Lerp(p2, p3, t2, t3, s, out a3, out da3);

                Vector3D b1, b2, db1, db2, ddb1, ddb2;
                Blend(a1, a2, da1, da2, t0, t2, s, out b1, out db1, out ddb1);
                Blend(a2, a3, da2, da3, t1, t3, s, out b2, out db2, out ddb2);

                Vector3D c, dc, ddc;
                Blend2(b1, b2, db1, db2, ddb1, ddb2, t1, t2, s, out c, out dc, out ddc);

                double scale = t2 - t1;
                firstDeriv = dc * scale;
                secondDeriv = ddc * (scale * scale);
                return c;
            }

            static void Lerp(Vector3D pa, Vector3D pb, double ta, double tb, double s,
                out Vector3D val, out Vector3D dval)
            {
                double inv = 1.0 / (tb - ta);
                double w = (tb - s) * inv;
                val = w * pa + (1.0 - w) * pb;
                dval = (pb - pa) * inv;
            }

            static void Blend(Vector3D a, Vector3D b, Vector3D da, Vector3D db,
                double ta, double tb, double s,
                out Vector3D val, out Vector3D dval, out Vector3D ddval)
            {
                double inv = 1.0 / (tb - ta);
                double wa = (tb - s) * inv;
                double wb = (s - ta) * inv;
                val = wa * a + wb * b;
                dval = (-inv) * a + wa * da + inv * b + wb * db;
                ddval = (2.0 * -inv) * da + (2.0 * inv) * db;
            }

            static void Blend2(Vector3D a, Vector3D b, Vector3D da, Vector3D db,
                Vector3D dda, Vector3D ddb, double ta, double tb, double s,
                out Vector3D val, out Vector3D dval, out Vector3D ddval)
            {
                double inv = 1.0 / (tb - ta);
                double wa = (tb - s) * inv;
                double wb = (s - ta) * inv;
                val = wa * a + wb * b;
                dval = (-inv) * a + wa * da + inv * b + wb * db;
                ddval = (2.0 * -inv) * da + wa * dda
                      + (2.0 * inv) * db + wb * ddb;
            }

            // Find u minimising distance to pos via a few Gauss-Newton steps from uHint.
            public double ClosestU(Vector3D pos, double uHint)
            {
                double u = uHint;
                if (u < 0.0) u = 0.0;
                if (u > SegmentCount) u = SegmentCount;
                for (int i = 0; i < 8; i++)
                {
                    Vector3D t;
                    Vector3D p = Position(u, out t);
                    double tMag2 = t.LengthSquared();
                    if (tMag2 < 1e-9) break;
                    double du = Vector3D.Dot(pos - p, t) / tMag2;
                    if (du < -0.5) du = -0.5;
                    if (du > 0.5) du = 0.5;
                    double next = u + du;
                    if (next < 0.0) next = 0.0;
                    if (next > SegmentCount) next = SegmentCount;
                    if (Math.Abs(next - u) < 1e-5) { u = next; break; }
                    u = next;
                }
                return u;
            }

            // Curvature kappa at u (1/m): |r' x r''| / |r'|^3.
            public double CurvatureAt(double u)
            {
                if (u < 0.0) u = 0.0;
                if (u > SegmentCount) u = SegmentCount;
                int seg = (int)Math.Floor(u);
                if (seg >= SegmentCount) seg = SegmentCount - 1;
                double t = u - seg;
                Vector3D firstDeriv, secondDeriv;
                EvalSegment(seg, t, out firstDeriv, out secondDeriv);
                double fdMag2 = firstDeriv.LengthSquared();
                if (fdMag2 < 1e-12) return 0.0;
                return Vector3D.Cross(firstDeriv, secondDeriv).Length() / Math.Pow(fdMag2, 1.5);
            }

            // Curvature with its turn axis (unit binormal B = normalize(T x d2)) at u.
            public double CurvatureWithAxis(double u, out Vector3D binormal)
            {
                binormal = Vector3D.Zero;
                if (u < 0.0) u = 0.0;
                if (u > SegmentCount) u = SegmentCount;
                int seg = (int)Math.Floor(u);
                if (seg >= SegmentCount) seg = SegmentCount - 1;
                double t = u - seg;
                Vector3D firstDeriv, secondDeriv;
                EvalSegment(seg, t, out firstDeriv, out secondDeriv);
                double fdMag2 = firstDeriv.LengthSquared();
                if (fdMag2 < 1e-12) return 0.0;
                Vector3D cross = Vector3D.Cross(firstDeriv, secondDeriv);
                double crossLen = cross.Length();
                if (crossLen > 1e-12) binormal = cross / crossLen;
                return crossLen / Math.Pow(fdMag2, 1.5);
            }
        }
    }
}
