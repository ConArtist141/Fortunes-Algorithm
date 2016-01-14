/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: A class which constructs a voronoi graph from a set of input
 * points. The voronoi graph is generated using Fortune's algorithm.
 *******************************************************************************/

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

    /// <summary>
    /// An event in the algorithm's vertical line sweep.
    /// </summary>
    class VoronoiEvent : PriorityQueueNode
    {
        /// <summary>
        /// The type of event.
        /// </summary>
        public VoronoiEventType Type;
        /// <summary>
        /// The Y-position this event occurs at, used for priority queue sorting.
        /// </summary>
        public double LocationY;
        /// <summary>
        /// If this is a circle event, this is the center of the circle in question.
        /// </summary>
        public Vector2d CircleCenter;
        /// <summary>
        /// An array of the sites involved. If this is a site event, then this simply
        /// contains the one site involed. Otherwise, if this is a circle event, it
        /// contains the site of the arc which will vanish and the sites of its two 
        /// neighboring arcs in left to right order along the beach line.
        /// </summary>
        public VoronoiSiteFace[] Sites;
        /// <summary>
        /// If this is a circle event, this is the vanishing arc.
        /// </summary>
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

    /// <summary>
    /// An object to represent an arc on the beach line.
    /// </summary>
    class VoronoiArcObject
    {
        /// <summary>
        /// The site associated with this arc.
        /// </summary>
        public VoronoiSiteFace Site;
        /// <summary>
        /// The circle event associated with this arc (if any).
        /// </summary>
        public VoronoiEvent Event;

        public override string ToString()
        {
            return "ID: " + Site.ID;
        }
    }

    /// <summary>
    /// An interface to a status structure which can be used to maintain the beach line.
    /// </summary>
    interface IVoronoiStatusStructure
    {
        /// <summary>
        /// Gets or sets the voronoi graph containing the half-edges used as separators in the line.
        /// </summary>
        VoronoiGraph Graph { get; set; }

        /// <summary>
        /// Gets whether this structure is empty.
        /// </summary>
        bool IsEmpty { get; }

        /// <summary>
        /// If the structure is empty, this can be used to insert the first arc.
        /// </summary>
        /// <param name="obj">Arc to insert</param>
        void Insert(VoronoiArcObject obj);

        /// <summary>
        /// Get the immediate neighborhood of an arc.
        /// </summary>
        /// <param name="obj">The arc in question</param>
        /// <param name="prevPrev">The arc two places to the left on the beach line</param>
        /// <param name="prev">The arc one place to the left on the beach line</param>
        /// <param name="succ">The arc one place to the right on the beach line</param>
        /// <param name="succSucc">The arc two places to the right on the beach line</param>
        /// <param name="prevSplittingEdge">The splitting edge to the left on the beach line</param>
        /// <param name="succSplittingEdge">The splitting edge to the right on the beach line</param>
        void GetNeighborhood(VoronoiArcObject obj, out VoronoiArcObject prevPrev, out VoronoiArcObject prev,
            out VoronoiArcObject succ, out VoronoiArcObject succSucc,
            out HalfEdge prevSplittingEdge, out HalfEdge succSplittingEdge);

        /// <summary>
        /// Remove an arc from the beach line.
        /// </summary>
        /// <param name="obj">The arc to remove</param>
        /// <param name="replacementEdge">The separating edge to replace it with</param>
        void Remove(VoronoiArcObject obj, HalfEdge replacementEdge);

        /// <summary>
        /// Create a new arc by splitting a current one on the beach line.
        /// </summary>
        /// <param name="splitArc">The arc to split</param>
        /// <param name="newArc">The new arc to insert</param>
        /// <param name="splittingEdgeCurrentSide">The splitting edge on the left of the new arc</param>
        /// <param name="splittingEdgeNewSide">The splitting edge on the right of the new arc</param>
        void Split(VoronoiArcObject splitArc, VoronoiArcObject newArc, 
            HalfEdge splittingEdgeCurrentSide, HalfEdge splittingEdgeNewSide);

        /// <summary>
        /// Find the arc above the specified point.
        /// </summary>
        /// <param name="point">The point to search with</param>
        /// <param name="yLine">The Y-position of the sweep line</param>
        /// <returns>The arc above the point specified</returns>
        VoronoiArcObject FindArcAbove(Vector2d point, double yLine);
    }

    /// <summary>
    /// An interface to a class which can construct a voronoi graph.
    /// </summary>
    interface IVoronoiConstructor
    {
        VoronoiGraph CreateVoronoi(List<Vector2d> points);
    }

    /// <summary>
    /// Default voronoi constructor.
    /// </summary>
    /// <typeparam name="StatusClass">The status object type used to represent the beach line</typeparam>
    class VoronoiConstructor<StatusClass> : IVoronoiConstructor
        where StatusClass : IVoronoiStatusStructure, new()
    {
        public static double CCW(Vector2d a, Vector2d b, Vector2d c)
        {
            return new Matrix3d(a.X, a.Y, 1.0, b.X, b.Y, 1.0, c.X, c.Y, 1.0).Determinant;
        }

        // Process the next event in the event queue and handle it appropriately
        private void ProcessEventQueue(HeapPriorityQueue<VoronoiEvent> queue, IVoronoiStatusStructure status, VoronoiGraph graph)
        {
            var queueEvent = queue.Dequeue();
            if (queueEvent.Type == VoronoiEventType.SiteEvent)
                HandleSiteEvent(queue, status, graph, queueEvent);
            else
                HandleCircleEvent(queue, status, graph, queueEvent);
        }

        /// <summary>
        /// Compute whether three adjacent arcs on the beach line will trigger a circle event, and register the event if necessary.
        /// </summary>
        /// <param name="arc1">The left arc</param>
        /// <param name="arc2">The middle arc</param>
        /// <param name="arc3">The right arc</param>
        /// <param name="queue">The event queue</param>
        /// <param name="yCutoff">The current Y-position of the sweep line</param>
        private void ComputeCircleEvent(VoronoiArcObject arc1, VoronoiArcObject arc2, 
            VoronoiArcObject arc3, HeapPriorityQueue<VoronoiEvent> queue, double yCutoff)
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

        // Handle a site event off of the event queue
        private void HandleSiteEvent(HeapPriorityQueue<VoronoiEvent> queue, IVoronoiStatusStructure status, 
            VoronoiGraph graph, VoronoiEvent evnt)
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

        // Handle a circle event off of the event queue
        private void HandleCircleEvent(HeapPriorityQueue<VoronoiEvent> queue, 
            IVoronoiStatusStructure status, VoronoiGraph graph, VoronoiEvent evnt)
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
            // IVoronoiStatusStructure status = new DefaultStatus(graph);
            IVoronoiStatusStructure status = new StatusClass();
            status.Graph = graph;

            // Insert all the events and faces
            for (int i = 0, count = points.Count; i < count; ++i)
            {
                var site = new VoronoiSiteFace() { ID = i, Edge = -1, Location = points[i] };
                graph.Faces.Add(site);
                queue.Enqueue(VoronoiEvent.CreateSiteEvent(site), -site.Location.Y);
            }

            // While there are still events in the queue, resolve them
            while (queue.Count != 0)
                ProcessEventQueue(queue, status, graph);

            return graph;
        }
    }
}
