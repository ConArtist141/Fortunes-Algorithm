using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;

namespace CompGeoProject
{
    class NaiveStatus : IVoronoiStatusStructure
    {
        private List<HalfEdge> separators = new List<HalfEdge>();
        private List<VoronoiArcObject> arcs = new List<VoronoiArcObject>();
        private VoronoiGraph graph;

        public NaiveStatus(VoronoiGraph graph)
        {
            this.graph = graph;
        }

        public bool IsEmpty
        {
            get
            {
                return arcs.Count == 0;
            }
        }

        public VoronoiArcObject FindArcAbove(Vector2d point, double yLine)
        {
            int i = 0;
            for (var count = separators.Count; i < count; ++i)
            {
                var sep = separators[i];
                var site1 = graph.Faces[sep.Face].Location;
                var site2 = graph.Faces[graph.Edges[sep.Opposite].Face].Location;
                var intersection = Line.ComputeEquidistantPoint(site1, site2, yLine);
                if (intersection.X >= point.X)
                    break;
            }

            return arcs[i];
        }

        public void GetNeighborhood(VoronoiArcObject obj, out VoronoiArcObject prevPrev, out VoronoiArcObject prev, out VoronoiArcObject succ, 
            out VoronoiArcObject succSucc, out HalfEdge prevSplittingEdge, out HalfEdge succSplittingEdge)
        {
            var i = arcs.IndexOf(obj);
            prev = (i <= 0 ? null : arcs[i - 1]);
            prevPrev = (i <= 1 ? null : arcs[i - 2]);
            succ = (i >= arcs.Count - 1 ? null : arcs[i + 1]);
            succSucc = (i >= arcs.Count - 2 ? null : arcs[i + 2]);
            prevSplittingEdge = (i <= 0 ? null : separators[i - 1]);
            succSplittingEdge = (i >= arcs.Count - 1 ? null : separators[i]);
        }

        public void Insert(VoronoiArcObject obj)
        {
            arcs.Add(obj);
        }

        public void Remove(VoronoiArcObject obj, HalfEdge replacementEdge)
        {
            var i = arcs.IndexOf(obj);
            arcs.RemoveAt(i);
            separators[i - 1] = replacementEdge;
            separators.RemoveAt(i);
        }

        public void Split(VoronoiArcObject currentObj, VoronoiArcObject newObj, HalfEdge splittingEdgeCurrentSide,
            HalfEdge splittingEdgeNewSide)
        {
            var i = arcs.IndexOf(currentObj);
            arcs.InsertRange(i + 1, new VoronoiArcObject[] { newObj,
                new VoronoiArcObject() { Site = arcs[i].Site } });
            separators.InsertRange(i, new HalfEdge[] { splittingEdgeCurrentSide, splittingEdgeNewSide });
        }
    }
}
