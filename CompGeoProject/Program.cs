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

namespace CompGeoProject
{
    class Program
    {
        static void Main(string[] args)
        {
            using (var window = new VoronoiWindow())
                window.Run();
        }
    }
}
