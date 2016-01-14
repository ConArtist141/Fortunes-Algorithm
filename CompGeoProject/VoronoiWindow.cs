/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: Main application code of my program. The voronoi window uses
 * OpenGL to display the results of the voronoi graph computation. The key
 * 'Space' can be used to recompute a new random graph. The key 'V' can be used
 * to toggle the display of vertices in the voronoi graph. The key 'S' can be
 * used to toggle the display of sites in the voronoi graph.
 *******************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Drawing;

using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Platform;
using OpenTK.Input;

namespace CompGeoProject
{
    /// <summary>
    /// An enumeration which specified which status structure to used for the beach line.
    /// </summary>
    enum AlgorithmVersion
    {
        Naive,
        BalancedBinaryTree
    }

    /// <summary>
    /// An OpenTK window used to render the result of computation
    /// </summary>
    class VoronoiWindow : GameWindow
    {
        /// <summary>
        /// The graph to be rendered.
        /// </summary>
        protected VoronoiGraph graph;
        /// <summary>
        /// The point size of a site.
        /// </summary>
        public const float SiteSize = 7.0f;
        /// <summary>
        /// The point size of a vertex.
        /// </summary>
        public const float VertexSize = 5.0f;
        /// <summary>
        /// Whether or not to display the vertices of the graph.
        /// </summary>
        protected bool bDisplayVertices = false;
        /// <summary>
        /// Whether or not to display the sites of the graph.
        /// </summary>
        protected bool bDisplaySites = true;
        /// <summary>
        /// The number of random sites to generate.
        /// </summary>
        public const int SiteCount = 100;
        /// <summary>
        /// The algorithm to use. Should be set to balanced binary tree for optimal performance.
        /// </summary>
        public AlgorithmVersion Algorithm = AlgorithmVersion.BalancedBinaryTree;

        public VoronoiWindow()
            : base(800, 600, new OpenTK.Graphics.GraphicsMode(new OpenTK.Graphics.ColorFormat(8), 24, 8, 8))
        {
            Title = "Voronoi Diagrams: Fortune's Algorithm";

            // Handle keyboard actions
            Keyboard.KeyDown += (object o, KeyboardKeyEventArgs e) =>
            {
                if (e.Key == Key.Escape)
                    Close();
                if (e.Key == Key.V)
                    bDisplayVertices = !bDisplayVertices;
                if (e.Key == Key.S)
                    bDisplaySites = !bDisplaySites;
                if (e.Key == Key.Space)
                    GenerateGraph();
            };
        }

        /// <summary>
        /// Generate a voronoi graph from a set of randomly generated points
        /// </summary>
        protected void GenerateGraph()
        {
            var random = new Random();

            // Generate random points in the [-1, 1] x [-1, 1] range
            var pts = (from i in Enumerable.Range(0, SiteCount)
                       select new Vector2d(random.NextDouble() * 2.0 - 1.0, random.NextDouble() * 2.0 - 1.0)).ToList();

            // Create the voronoi constructor
            IVoronoiConstructor constructor;
            if (Algorithm == AlgorithmVersion.BalancedBinaryTree)
                constructor = new VoronoiConstructor<BalancedBinaryTreeStatus>();
            else
                constructor = new VoronoiConstructor<NaiveStatus>();

            // Compute the voronoi graph
            var graphResult = constructor.CreateVoronoi(pts);
            // Cap any infinite edges
            graphResult.Complete(2f);

            graph = graphResult;
        }

        protected override void OnLoad(EventArgs e)
        {
            // Generate the graph which the application starts up
            GenerateGraph();

            GL.Enable(EnableCap.ProgramPointSize);
        }

        protected override void OnResize(EventArgs e)
        {
            GL.Viewport(ClientSize);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            GL.ClearColor(Color.White);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);

            // Draw sites
            if (bDisplaySites)
            {
                GL.PointSize(SiteSize);
                GL.Begin(PrimitiveType.Points);
                foreach (var face in graph.Faces)
                {
                    GL.Color3(Color.Purple);
                    GL.Vertex2(face.Location);
                }
                GL.End();
            }

            // Draw edges of voronoi graph
            GL.Begin(PrimitiveType.Lines);
            foreach (var edge in graph.Edges)
            {
                if (edge.Vertex == -1 || graph.Edges[edge.Opposite].Vertex == -1)
                    continue;

                var vert1 = graph.Vertices[edge.Vertex].Position;
                var vert2 = graph.Vertices[graph.Edges[edge.Opposite].Vertex].Position;

                GL.Vertex2(vert1);
                GL.Color3(Color.Black);
                GL.Vertex2(vert2);
                GL.Color3(Color.Black);
            }
            GL.End();

            // Draw vertices of the voronoi graph
            if (bDisplayVertices)
            {
                GL.PointSize(VertexSize);
                GL.Begin(PrimitiveType.Points);

                foreach (var vertex in graph.Vertices)
                {
                    GL.Color3(Color.Black);
                    GL.Vertex2(vertex.Position);
                }
                GL.End();
            }

            // Display result
            SwapBuffers();
        }
    }
}
