using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Drawing;

using OpenTK;
/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: This file contains the main application code
 *******************************************************************************/
using OpenTK.Graphics.OpenGL;
using OpenTK.Platform;
using OpenTK.Input;

namespace CompGeoProject
{
    enum AlgorithmVersion
    {
        Naive,
        BalancedBinaryTree
    }

    class VoronoiWindow : GameWindow
    {
        protected VoronoiGraph graph;
        public const float SiteSize = 7.0f;
        public const float VertexSize = 5.0f;
        protected bool bDisplayVertices = false;
        protected bool bDisplaySites = true;
        public const int PointCount = 100;
        public AlgorithmVersion Algorithm = AlgorithmVersion.BalancedBinaryTree;

        public VoronoiWindow()
            : base(800, 600, new OpenTK.Graphics.GraphicsMode(new OpenTK.Graphics.ColorFormat(8), 24, 8, 8))
        {
            Title = "Voronoi Diagrams: Fortune's Algorithm";

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

        protected void GenerateGraph()
        {
            var random = new Random();

            var pts = (from i in Enumerable.Range(0, PointCount) select new Vector2d(random.NextDouble() * 2.0 - 1.0, random.NextDouble() * 2.0 - 1.0)).ToList();

            IVoronoiConstructor constructor;
            if (Algorithm == AlgorithmVersion.BalancedBinaryTree)
                constructor = new VoronoiConstructor<BalancedBinaryTreeStatus>();
            else
                constructor = new VoronoiConstructor<NaiveStatus>();

            var graphResult = constructor.CreateVoronoi(pts);
            graphResult.Complete(2f);

            graph = graphResult;
        }

        protected override void OnLoad(EventArgs e)
        {
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

            SwapBuffers();
        }
    }
}
