/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: Entry point for the application.
 *******************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;

using OpenTK;

namespace CompGeoProject
{
    class Program
    {
        /// <summary>
        /// Entry point. Argument 0 should either be a number or the string "perf."
        /// A number specifies how many sites to use for the voronoi graph.
        /// "perf" tells the application to run performance tests.
        /// </summary>
        /// <param name="args">Argument collection</param>
        static void Main(string[] args)
        {
            int siteCount = VoronoiWindow.DefaultSiteCount;
            bool bRunPerformanceTests = false;

            if (args.Length > 0)
                if (args[0] == "perf")
                    bRunPerformanceTests = true;
                else
                    int.TryParse(args[0], out siteCount);

            if (!bRunPerformanceTests)
            {
                using (var window = new VoronoiWindow(siteCount))
                    window.Run();
            }
            else
            {
                RunPerformanceTests();
            }
        }

        /// <summary>
        /// Runs a performance test on the algorithm
        /// </summary>
        static void RunPerformanceTests()
        {
            var runs = from i in Enumerable.Range(7, 15)
                       select 1L << i;

            var random = new Random();

            foreach (var run in runs)
            {
                var stopwatch = new Stopwatch();
                var constructor = new VoronoiConstructor<BalancedBinaryTreeStatus>();

                var pts = new Vector2d[run];
                for (int i = 0; i < run; ++i)
                    pts[i] = new Vector2d(random.NextDouble() * 2.0 - 1.0, random.NextDouble() * 2.0 - 1.0);

                stopwatch.Reset();
                stopwatch.Start();
                constructor.CreateVoronoi(pts);
                stopwatch.Stop();

                Console.WriteLine("Input Size: {0}, Time: {1} s", run, (double)stopwatch.ElapsedMilliseconds / 1000.0);
            }
        }
    }
}
