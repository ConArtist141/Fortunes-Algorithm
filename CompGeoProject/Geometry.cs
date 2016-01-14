/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: A handful of geometry tools.
 *******************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK;

namespace CompGeoProject
{
    /// <summary>
    /// A structure to represent a directed (parameterized) line.
    /// </summary>
    struct DirectedLine
    {
        public Vector2d Direction;
        public Vector2d Start;

        /// <summary>
        /// Gets the perpendicular bisector of two points.
        /// </summary>
        /// <param name="p1">First point</param>
        /// <param name="p2">Second point</param>
        /// <returns>The perpendicular bisector</returns>
        public static DirectedLine GetPerpendicularBisector(Vector2d p1, Vector2d p2)
        {
            var diff = p2 - p1;
            var result = Vector3d.Cross(new Vector3d(diff.X, diff.Y, 0.0f), new Vector3d(0.0f, 0.0f, 1.0f));
            result.Normalize();
            return new DirectedLine()
            {
                Direction = result.Xy,
                Start = p1 + diff / 2.0f
            };
        }

        /// <summary>
        /// Gets the intersection of two lines, one directed and the other undirected.
        /// </summary>
        /// <param name="line1">First line, directed</param>
        /// <param name="line2">Second line, undirected</param>
        /// <returns>The distance along the directed line the collision occurs</returns>
        public static double GetIntersection(DirectedLine line1, Line line2)
        {
            return (line2.Distance - Vector2d.Dot(line1.Start, line2.Normal)) / Vector2d.Dot(line1.Direction, line2.Normal);
        }

        /// <summary>
        /// Gets the intersection of two lines, one directed and the other undirected.
        /// </summary>
        /// <param name="line1">First line, directed</param>
        /// <param name="line2">Second line, undirected</param>
        /// <param name="time">The distance along the directed</param>
        /// <returns>Whether the two lines intersect</returns>
        public static bool GetIntersection(DirectedLine line1, Line line2, ref double time)
        {
            var res = Vector2d.Dot(line1.Direction, line2.Normal);
            if (res == 0.0f)
                return false;
            var p = Vector2d.Dot(line1.Start, line2.Normal);
            time = (line2.Distance - p) / res;
            return true;
        }

        /// <summary>
        /// Gets the position of a parameter value along this line.
        /// </summary>
        /// <param name="time">The parameter value</param>
        /// <returns>The position</returns>
        public Vector2d GetPosition(double time)
        {
            return Start + Direction * time;
        }
    }

    /// <summary>
    /// A structure to implicitly represent a line.
    /// </summary>
    struct Line
    {
        public Vector2d Normal;
        public double Distance;

        /// <summary>
        /// Get the perpendicular bisector of two points.
        /// </summary>
        /// <param name="p1">First point</param>
        /// <param name="p2">Second point</param>
        /// <returns>The resulting line</returns>
        public static Line GetPerpendicularBisector(Vector2d p1, Vector2d p2)
        {
            Vector2d diff = p1 - p2;
            diff.Normalize();
            var dist1 = Vector2d.Dot(p1, diff);
            var dist2 = Vector2d.Dot(p2, diff);
            return new Line() { Normal = diff, Distance = (dist1 + dist2) / 2.0d };
        }

        /// <summary>
        /// Get the intersection of two lines.
        /// </summary>
        /// <param name="l1">The first line</param>
        /// <param name="l2">The second line</param>
        /// <param name="position">The position of intersection</param>
        /// <returns>Whether or not the lines intersect</returns>
        public static bool GetIntersection(Line l1, Line l2, ref Vector2d position)
        {
            var matrix = new Matrix2d(l1.Normal, l2.Normal);
            if (matrix.Determinant == 0.0f)
                return false;

            matrix.Invert();
            var distVector = new Vector2d(l1.Distance, l2.Distance);
            position.X = Vector2d.Dot(matrix.Row0, distVector);
            position.Y = Vector2d.Dot(matrix.Row1, distVector);
            return true;
        }

        /// <summary>
        ///  Computes the point equidistant from two points and a line (given by y = yLine)
        /// </summary>
        /// <param name="p1">First point</param>
        /// <param name="p2">Second point</param>
        /// <param name="yLine">The y-value of the line</param>
        /// <returns>The resulting point</returns>
        public static Vector2d ComputeEquidistantPoint(Vector2d p1, Vector2d p2, double yLine)
        {
            var div1 = 1.0 / (2.0 * (p1.Y - yLine));
            var div2 = 1.0 / (2.0 * (p2.Y - yLine));
            var a1 = div1;
            var b1 = -2.0 * p1.X * div1;
            var c1 = (p1.X * p1.X + p1.Y * p1.Y - yLine * yLine) * div1;
            var a2 = div2;
            var b2 = -2.0 * p2.X * div2;
            var c2 = (p2.X * p2.X + p2.Y * p2.Y - yLine * yLine) * div2;
            var a3 = a1 - a2;
            var b3 = b1 - b2;
            var c3 = c1 - c2;
            var sqrt = Math.Sqrt(b3 * b3 - 4.0 * a3 * c3);
            var result1 = (-b3 - sqrt) / (2.0 * a3);
            var result2 = (-b3 + sqrt) / (2.0 * a3);
            var minresult = Math.Min(result1, result2);
            var maxresult = Math.Max(result1, result2);
            if (p1.Y > p2.Y)
                return new Vector2d(minresult, minresult * minresult * a1 + minresult * b1 + c1);
            else
                return new Vector2d(maxresult, maxresult * maxresult * a1 + maxresult * b1 + c1);
        }
    }
}