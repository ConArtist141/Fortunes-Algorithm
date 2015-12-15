using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK;
using Priority_Queue;

namespace CompGeoProject
{
    enum VoronoiEventType
    {
        SiteEvent,
        CircleEvent
    }

    class VoronoiEvent : PriorityQueueNode
    {
        public VoronoiEventType Type;
        public double LocationY;
        public Vector2d CircleCenter;
        public VoronoiSiteFace[] Sites;
        public VoronoiArcObject ArcObject;

        public static VoronoiEvent CreateSiteEvent(VoronoiSiteFace site)
        {
            return new VoronoiEvent()
            {
                Type = VoronoiEventType.SiteEvent,
                LocationY = site.Location.Y,
                Sites = new VoronoiSiteFace[] { site }
            };
        }

        public static VoronoiEvent CreateCircleEvent(VoronoiArcObject arc1, VoronoiArcObject arc2,
            VoronoiArcObject arc3, Vector2d center, double locationY)
        {
            return new VoronoiEvent()
            {
                CircleCenter = center,
                LocationY = locationY,
                Sites = new VoronoiSiteFace[] { arc1.Site, arc2.Site, arc3.Site },
                Type = VoronoiEventType.CircleEvent,
                ArcObject = arc2
            };
        }
    }

    class VoronoiArcObject
    {
        public VoronoiSiteFace Site;
        public VoronoiEvent Event;

        public override string ToString()
        {
            return "ID: " + Site.ID;
        }
    }

    interface IVoronoiStatusStructure
    {
        bool IsEmpty { get; }
        void Insert(VoronoiArcObject obj);
        void GetNeighborhood(VoronoiArcObject obj, out VoronoiArcObject prevPrev, out VoronoiArcObject prev,
            out VoronoiArcObject succ, out VoronoiArcObject succSucc,
            out HalfEdge prevSplittingEdge, out HalfEdge succSplittingEdge);
        void Remove(VoronoiArcObject obj, HalfEdge replacementEdge);
        void Split(VoronoiArcObject currentObj, VoronoiArcObject newObj, HalfEdge splittingEdgeCurrentSide, HalfEdge splittingEdgeNewSide);
        VoronoiArcObject FindArcAbove(Vector2d point, double yLine);
    }

    class VoronoiConstructor
    {
        public static double CCW(Vector2d a, Vector2d b, Vector2d c)
        {
            return new Matrix3d(a.X, a.Y, 1.0, b.X, b.Y, 1.0, c.X, c.Y, 1.0).Determinant;
        }

        private void ProcessEventQueue(List<Vector2d> points, HeapPriorityQueue<VoronoiEvent> queue, IVoronoiStatusStructure status, VoronoiGraph graph)
        {
            var queueEvent = queue.Dequeue();
            if (queueEvent.Type == VoronoiEventType.SiteEvent)
                HandleSiteEvent(points, queue, status, graph, queueEvent);
            else
                HandleCircleEvent(points, queue, status, graph, queueEvent);
        }

        private void ComputeCircleEvent(VoronoiArcObject arc1, VoronoiArcObject arc2, VoronoiArcObject arc3, HeapPriorityQueue<VoronoiEvent> queue, double yCutoff)
        {
            // Only register circle event if CCW is correct (otherwise, we may register twice)
            if (CCW(arc1.Site.Location, arc2.Site.Location, arc3.Site.Location) < 0.0)
            {
                var line1 = Line.GetPerpendicularBisector(arc1.Site.Location, arc2.Site.Location);
                var line2 = Line.GetPerpendicularBisector(arc2.Site.Location, arc3.Site.Location);
                var result = Vector2d.Zero;
                if (Line.GetIntersection(line1, line2, ref result))
                {
                    // Compute the center of the circle formed by these arcs
                    var dropDistance = (arc1.Site.Location - result).Length;

                    // Drop this center vertically, this is where the event will occur
                    var eventLocationY = result.Y - dropDistance;

                    // Check if circle event happens below the sweep line
                    if (eventLocationY <= yCutoff)
                    {
                        // There will be a circle event, so register it
                        arc2.Event = VoronoiEvent.CreateCircleEvent(arc1, arc2, arc3, result, eventLocationY);
                        queue.Enqueue(arc2.Event, -eventLocationY);
                    }
                }
            }
        }

        private void HandleSiteEvent(List<Vector2d> points, HeapPriorityQueue<VoronoiEvent> queue, IVoronoiStatusStructure status, VoronoiGraph graph, VoronoiEvent evnt)
        {
            var site = evnt.Sites[0];

            if (status.IsEmpty)
                status.Insert(new VoronoiArcObject() { Site = site, Event = null });
            else
            {
                // Find the arc above this site
                var arc = status.FindArcAbove(site.Location, evnt.LocationY);
                // If the arc has a circle event, then it is a false alarm, so remove it
                if (arc.Event != null)
                {
                    queue.Remove(arc.Event);
                    arc.Event = null;
                }

                // Split the arc which lies above this site
                var newArc = new VoronoiArcObject() { Site = site, Event = null };
                VoronoiArcObject leftArc, leftLeftArc, rightArc, rightRightArc;

                // Add edge records
                var edge1 = new HalfEdge() { ID = graph.Edges.Count };
                var edge2 = new HalfEdge() { ID = graph.Edges.Count + 1 };
                edge1.Opposite = edge2.ID;
                edge2.Opposite = edge1.ID;
                graph.Edges.Add(edge1);
                graph.Edges.Add(edge2);
                arc.Site.Edge = edge1.ID;
                edge1.Face = arc.Site.ID;
                newArc.Site.Edge = edge2.ID;
                edge2.Face = newArc.Site.ID;

                // Perform the split
                status.Split(arc, newArc, edge1, edge2);

                HalfEdge a1, a2;
                status.GetNeighborhood(newArc, out leftLeftArc, out leftArc,
                    out rightArc, out rightRightArc, out a1, out a2);

                // Register new events if necessary, make sure the same circle event isn't registered twice
                if (leftLeftArc != null)
                    ComputeCircleEvent(leftLeftArc, leftArc, newArc, queue, evnt.LocationY);
                if (rightRightArc != null)
                    ComputeCircleEvent(newArc, rightArc, rightRightArc, queue, evnt.LocationY);
            }
        }

        private void HandleCircleEvent(List<Vector2d> points, HeapPriorityQueue<VoronoiEvent> queue, IVoronoiStatusStructure status, VoronoiGraph graph, VoronoiEvent evnt)
        {
            // Remove the arc from the status structure
            VoronoiArcObject prevPrev, prev, succ, succSucc;
            HalfEdge prevSplittingEdge, succSplittingEdge;
            status.GetNeighborhood(evnt.ArcObject, out prevPrev, out prev, out succ, out succSucc,
                out prevSplittingEdge, out succSplittingEdge);

            // Remove circle events of the previous and succeeding arcs
            if (prev.Event != null)
            {
                queue.Remove(prev.Event);
                prev.Event = null;
            }
            if (succ.Event != null)
            {
                queue.Remove(succ.Event);
                succ.Event = null;
            }

            // Connect edges and create new edges and vertices
            var newEdge = new HalfEdge() { ID = graph.Edges.Count };
            var newEdgeOp = new HalfEdge() { ID = graph.Edges.Count + 1 };
            newEdge.Opposite = newEdgeOp.ID;
            newEdgeOp.Opposite = newEdge.ID;
            newEdge.Face = prev.Site.ID;
            newEdgeOp.Face = succ.Site.ID;

            // Connect edges to new vertex
            var newVertex = new Vertex()
            {
                Edge = newEdge.ID,
                ID = graph.Vertices.Count,
                Position = evnt.CircleCenter
            };

            var succSplittingOp = graph.Edges[succSplittingEdge.Opposite];
            var prevSplittingOp = graph.Edges[prevSplittingEdge.Opposite];
            newEdge.Vertex = newVertex.ID;
            prevSplittingOp.Vertex = newVertex.ID;
            succSplittingOp.Vertex = newVertex.ID;

            // Connect edges
            prevSplittingOp.Next = succSplittingEdge.ID;
            newEdge.Next = prevSplittingEdge.ID;
            succSplittingOp.Next = newEdgeOp.ID;

            graph.Vertices.Add(newVertex);
            graph.Edges.Add(newEdge);
            graph.Edges.Add(newEdgeOp);

            // Actually remove the arc from the status structure, add in new edge
            status.Remove(evnt.ArcObject, newEdge);

            // Register new circle events if necessary
            if (prevPrev != null)
                ComputeCircleEvent(prevPrev, prev, succ, queue, evnt.LocationY);
            if (succSucc != null)
                ComputeCircleEvent(prev, succ, succSucc, queue, evnt.LocationY);
        }

        public VoronoiGraph CreateVoronoi(List<Vector2d> points)
        {
            points.Sort((p1, p2) => (p1.Y > p2.Y) ? -1 : 1);

            var graph = new VoronoiGraph();
            var queue = new HeapPriorityQueue<VoronoiEvent>(points.Count * 10);
            IVoronoiStatusStructure status = new NaiveStatus(graph);

            // Insert all the events and faces
            for (int i = 0, count = points.Count; i < count; ++i)
            {
                var site = new VoronoiSiteFace() { ID = i, Edge = -1, Location = points[i] };
                graph.Faces.Add(site);
                queue.Enqueue(VoronoiEvent.CreateSiteEvent(site), -site.Location.Y);
            }

            while (queue.Count != 0)
                ProcessEventQueue(points, queue, status, graph);

            return graph;
        }
    }
}
