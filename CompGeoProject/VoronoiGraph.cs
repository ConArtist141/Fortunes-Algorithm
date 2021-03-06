﻿/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: My implementation of the half-edge data structure.
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
    /// A face in a voronoi graph, corresponding to a voronoi site.
    /// </summary>
    class VoronoiSiteFace
    {
        public int ID = -1;
        public int Edge = -1;
        public Vector2d Location;

        public override string ToString()
        {
            return "ID: " + ID;
        }
    }

    /// <summary>
    /// A half edge in a voronoi graph.
    /// </summary>
    class HalfEdge
    {
        public int ID = -1;
        public int Face = -1;
        public int Opposite = -1;
        public int Next = -1;
        public int Vertex = -1;

        public override string ToString()
        {
            return "ID: " + ID;
        }
    }

    /// <summary>
    /// A vertex in a voronoi graph.
    /// </summary>
    class Vertex
    {
        public int ID = -1;
        public int Edge = -1;
        public Vector2d Position;

        public override string ToString()
        {
            return "ID: " + ID;
        }
    }

    /// <summary>
    /// A half-edge representation of a voronoi graph.
    /// </summary>
    class VoronoiGraph
    {
        public List<VoronoiSiteFace> Faces = new List<VoronoiSiteFace>();
        public List<Vertex> Vertices = new List<Vertex>();
        public List<HalfEdge> Edges = new List<HalfEdge>();

        /// <summary>
        /// Cap any infinite edges to a finite region.
        /// </summary>
        /// <param name="extensionLength">The maximum amount to extent an edge by</param>
        public void Complete(double extensionLength)
        {
            for (int i = 0, count = Edges.Count; i < count; ++i)
            {
                var edge = Edges[i];

                // Find edges without vertices
                if (edge.Vertex == -1)
                {
                    // Cap these edges
                    edge.Next = edge.Opposite;
                    var face = Faces[edge.Face];
                    var op = Edges[edge.Opposite];
                    var opFace = Faces[op.Face];
                    var line = DirectedLine.GetPerpendicularBisector(opFace.Location, face.Location);
                    var pos = line.GetPosition(extensionLength);
                    var vert = new Vertex() { Edge = op.ID, ID = Vertices.Count, Position = pos };
                    edge.Vertex = vert.ID;
                    Vertices.Add(vert);
                }
            }
        }
    }
}
