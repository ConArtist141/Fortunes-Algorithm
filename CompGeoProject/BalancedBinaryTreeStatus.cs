/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: This is the balanced binary tree data structure which I use
 * to maintain the beach line of parabolas.
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
    /// <summary>
    /// A node in the balanced binary tree. The node may either be an arc node or
    /// a separator node. Arc nodes must be leaves of the tree and separator nodes
    /// must be in the interior.
    /// </summary>
    class TreeNode
    {
        /// <summary>
        /// An arc in the beach line, if this is not null, then this node is an arc node.
        /// </summary>
        public VoronoiArcObject Arc;
        /// <summary>
        /// A separator (half-edge) between two sites, if this is not null, then this node is a separator node.
        /// </summary>
        public HalfEdge Separator;
        /// <summary>
        /// The rank of this node in the tree. Used for balancing.
        /// </summary>
        public int Rank = 0;
        /// <summary>
        /// If this is a separator node, then this is the left child. Otherwise, it is the previous arc on the beach line.
        /// </summary>
        public TreeNode Left;
        /// <summary>
        /// If this is a separator node, then this is the right child. Otherwise, it is the next arc on the beach line.
        /// </summary>
        public TreeNode Right;
        /// <summary>
        /// The parent of this node.
        /// </summary>
        public TreeNode Parent;

        public static TreeNode CreateArcNode(VoronoiArcObject arcObject, TreeNode parent, TreeNode prev, TreeNode next)
        {
            return new TreeNode()
            {
                Arc = arcObject,
                Left = prev,
                Right = next,
                Parent = parent,
                Separator = null
            };
        }

        public static TreeNode CreateSeparatorNode(HalfEdge separator, TreeNode parent, TreeNode left, TreeNode right)
        {
            return new TreeNode()
            {
                Separator = separator,
                Left = left,
                Right = right,
                Parent = parent,
                Arc = null
            };
        }

        public override string ToString()
        {
            if (Arc != null)
                return String.Format("Arc: {0}", Arc);
            else
                return String.Format("Separator: {0}", Separator);
        }
    }

    /// <summary>
    /// A beach line status structure which uses a balanced binary tree internally. Nodes in the tree are either arc nodes or
    /// separator nodes corresponding to splitting lines between arcs. The leaves of the tree are all arc nodes and they form
    /// a doubly linked list at the bottom of the tree. The way the tree is balanced is similar to the AA tree, but since
    /// there are very strict rules regarding when nodes are inserted and deleted, the AA tree data structure has been modified
    /// for this particular situtation.
    /// </summary>
    class BalancedBinaryTreeStatus : IVoronoiStatusStructure
    {
        /// <summary>
        /// The root of the balanced binary tree.
        /// </summary>
        private TreeNode Root;
        /// <summary>
        /// A map from arcs on the beach line to nodes in the tree. Allows O(lg n) lookup of arcs.
        /// </summary>
        private Dictionary<VoronoiArcObject, TreeNode> LeafMap = new Dictionary<VoronoiArcObject, TreeNode>();
        /// <summary>
        /// The voronoi graph under construction.
        /// </summary>
        private VoronoiGraph graph;
        /// <summary>
        /// Used for debug purposes only, number of calls to CheckDebug.
        /// </summary>
        private int debugCalls = 0;

        public bool IsEmpty
        {
            get
            {
                return Root == null;
            }
        }

        public VoronoiGraph Graph
        {
            get { return graph; }
            set { graph = value; }
        }

        public void Insert(VoronoiArcObject obj)
        {
            // Create root
            Root = TreeNode.CreateArcNode(obj, null, null, null);

            // Update leaf map
            LeafMap[Root.Arc] = Root;
        }

        /// <summary>
        /// Find the separating edges on both sides of the arc node specified.
        /// </summary>
        /// <param name="arcNode">The query node</param>
        /// <param name="leftSeparator">The separating edge on the left</param>
        /// <param name="rightSeparator">The separating edge on the right</param>
        protected void FindAdjacentSeparators(TreeNode arcNode, out TreeNode leftSeparator, out TreeNode rightSeparator)
        {
            var sep1 = arcNode.Parent;
            Debug.Assert(sep1.Left == arcNode || sep1.Right == arcNode);

            leftSeparator = null;
            rightSeparator = null;

            if (sep1.Left == arcNode)
            {
                // Go up until we take our first left turn
                rightSeparator = sep1;
                for (TreeNode node = sep1.Parent, prevNode = sep1; node != null;
                    prevNode = node, node = node.Parent)
                    if (node.Right == prevNode)
                    {
                        leftSeparator = node;
                        break;
                    }
            }
            else
            {
                // Go up until we take our first right
                leftSeparator = sep1;
                for (TreeNode node = sep1.Parent, prevNode = sep1; node != null;
                    prevNode = node, node = node.Parent)
                    if (node.Left == prevNode)
                    {
                        rightSeparator = node;
                        break;
                    }
            }
        }

        public void GetNeighborhood(VoronoiArcObject obj, out VoronoiArcObject prevPrev, 
            out VoronoiArcObject prev, out VoronoiArcObject succ,
            out VoronoiArcObject succSucc, out HalfEdge prevSplittingEdge, 
            out HalfEdge succSplittingEdge)
        {
            var node = LeafMap[obj];
            prev = node.Left?.Arc;
            prevPrev = node.Left?.Left?.Arc;
            succ = node.Right?.Arc;
            succSucc = node.Right?.Right?.Arc;

            // Find the separators on either side of this arc node
            TreeNode leftSep;
            TreeNode rightSep;
            FindAdjacentSeparators(node, out leftSep, out rightSep);

            prevSplittingEdge = leftSep?.Separator;
            succSplittingEdge = rightSep?.Separator;
        }

        /// <summary>
        /// Get all of the separator nodes of this tree with an in order traversal.
        /// </summary>
        /// <param name="root">The root at which to start</param>
        /// <returns>An enumerable of the separator nodes</returns>
        private IEnumerable<TreeNode> GetSeparatorsInOrder(TreeNode root)
        {
            if (root.Separator != null)
            {
                foreach (var sep in GetSeparatorsInOrder(root.Left))
                    yield return sep;
                yield return root;
                foreach (var sep in GetSeparatorsInOrder(root.Right))
                    yield return sep;
            }
        }

        /// <summary>
        /// Checks the validity of the tree structure and outputs the contents of the tree.
        /// </summary>
        private void DebugCheck()
        {
            Console.WriteLine("Debug Check {0}: ", debugCalls++);
            var node = Root;
            for (; node.Arc == null; node = node.Left) ;
            Debug.Assert(node.Left == null);
            Debug.Assert(node.Separator == null);

            var current = node.Right;
            Console.WriteLine("Arcs: ");
            Console.Write("{0} ", node.Arc);
            for (var prev = node; current != null; prev = current, current = current.Right)
            {
                Debug.Assert(prev.Right == current);
                Debug.Assert(current.Left == prev);
                Debug.Assert(prev.Separator == null);
                Debug.Assert(current.Separator == null);
                Debug.Assert(prev.Arc != null);
                Debug.Assert(current.Arc != null);
                Console.Write("{0} ", current.Arc);
            }
            Console.WriteLine();
            Console.WriteLine("Separators: ");
            foreach (var sep in GetSeparatorsInOrder(Root))
            {
                Debug.Assert(sep.Arc == null);
                Debug.Assert(sep.Separator != null);
                Debug.Assert(sep.Left != null);
                Debug.Assert(sep.Right != null);
                Console.Write("{0} ", sep.Separator);
            }
            Console.WriteLine();
        }

        public void Remove(VoronoiArcObject obj, HalfEdge replacementEdge)
        {
            /* To peform a removal, we need to find the higher of the two
            separator nodes for the splitting lines surrounding obj. The
            higher one can them be overwritten by the replacement edge.
            The lower one is removed from the tree and the twin of the
            arc obj is moved up in the tree to replace it. */
             
            // Find the high separator
            var node = LeafMap[obj];

            Debug.Assert(node.Parent.Parent != null);

            TreeNode leftSep;
            TreeNode rightSep;
            FindAdjacentSeparators(node, out leftSep, out rightSep);
            Debug.Assert(leftSep != null && rightSep != null);

            TreeNode highNode;
            var lowNode = node.Parent;
            if (lowNode == rightSep)
                highNode = leftSep;
            else
                highNode = rightSep;

            // Replace the high separator node 
            highNode.Separator = replacementEdge;

            var parent = node.Parent;

            // Move the twin arc node up to replace the separator node above
            var twin = (parent.Left == node ? parent.Right : parent.Left);
            twin.Parent = parent.Parent;
            if (parent.Parent.Left == parent)
                parent.Parent.Left = twin;
            else
                parent.Parent.Right = twin;

            parent.Separator = null;
            parent.Left = null;
            parent.Right = null;

            // Fix linked list
            if (node.Left != null)
                node.Left.Right = node.Right;
            if (node.Right != null)
                node.Right.Left = node.Left;

            // Clean up leaf map
            LeafMap.Remove(obj);

            // Propogate rank difference correctly + balance tree
            PropogateRankOnDelete(twin);
        }

        public void Split(VoronoiArcObject currentObj, VoronoiArcObject newObj, 
            HalfEdge splittingEdgeCurrentSide, HalfEdge splittingEdgeNewSide)
        {
            var node = LeafMap[currentObj];

            var rightArc = new VoronoiArcObject() { Site = currentObj.Site };

            var prevArcNode = node.Left;
            var nextArcNode = node.Right;

            // Split the specified arc and create a new substructure in the tree
            var leftMiddleSeparatorNode = TreeNode.CreateSeparatorNode(splittingEdgeCurrentSide, node, null, null);
            var rightArcNode = TreeNode.CreateArcNode(rightArc, node, null, nextArcNode);
            var leftArcNode = TreeNode.CreateArcNode(currentObj, leftMiddleSeparatorNode, prevArcNode, null);
            var middleArcNode = TreeNode.CreateArcNode(newObj, leftMiddleSeparatorNode, leftArcNode, rightArcNode);
            rightArcNode.Left = middleArcNode;
            leftArcNode.Right = middleArcNode;

            leftMiddleSeparatorNode.Left = leftArcNode;
            leftMiddleSeparatorNode.Right = middleArcNode;

            node.Arc = null;
            node.Left = leftMiddleSeparatorNode;
            node.Right = rightArcNode;
            node.Separator = splittingEdgeNewSide;

            // Set ranks
            node.Rank = 1;
            leftMiddleSeparatorNode.Rank = 1;

            // Update leaf map
            LeafMap[currentObj] = leftArcNode;
            LeafMap[newObj] = middleArcNode;
            LeafMap[rightArc] = rightArcNode;

            // Fix linked list
            if (prevArcNode != null)
                prevArcNode.Right = leftArcNode;
            if (nextArcNode != null)
                nextArcNode.Left = rightArcNode;

            PropogateRankOnInsert(leftMiddleSeparatorNode);
        }

        /// <summary>
        /// Propogates rank and rebalances the tree on a deletion.
        /// </summary>
        /// <param name="startNode">The (0, 0)-separator node to start at</param>
        private void PropogateRankOnDelete(TreeNode startNode)
        {
            for (TreeNode current = startNode.Parent; current != null; current = current.Parent)
            {
                var newRank = Math.Max(current.Left.Rank, current.Right.Rank - 1) + 1;
                current.Rank = newRank;

                current = BalanceSplit(current);
            }
        }

        /// <summary>
        /// Propogates rank and rebalances the tree on an insertion.
        /// </summary>
        /// <param name="startNode">The starting node to begin rebalancing</param>
        private void PropogateRankOnInsert(TreeNode startNode)
        {
            // Propogate rank difference correctly + balance tree
            for (TreeNode current = startNode.Parent; current != null; current = current.Parent)
            {
                current = BalanceSkew(current);
                current = BalanceSplit(current);
            }
        }

        /// <summary>
        /// Perform an AA-split on the specified node.
        /// </summary>
        /// <param name="node">Node on which to perform split</param>
        /// <returns>Top node after rotation</returns>
        private TreeNode BalanceSplit(TreeNode node)
        {
            if (node.Right.Arc != null || node.Right.Right == null || node.Right.Right.Arc != null)
                return node;
            else if (node.Rank == node.Right.Right.Rank)
            {
                // Perform an AA Split
                var right = node.Right;
                var rightRight = node.Right.Right;
                var rightLeft = node.Right.Left;

                node.Right = rightLeft;
                right.Left = node;

                if (node.Parent != null)
                {
                    if (node.Parent.Left == node)
                        node.Parent.Left = right;
                    else
                        node.Parent.Right = right;
                }
                else
                    Root = right;

                rightLeft.Parent = node;
                right.Parent = node.Parent;
                node.Parent = right;

                right.Rank++;

                return right;
            }
            else
                return node;
        }

        /// <summary>
        /// Perform an AA-skew on the specified node.
        /// </summary>
        /// <param name="node">The node on which to perform a skew</param>
        /// <returns>Top node after rotation</returns>
        private TreeNode BalanceSkew(TreeNode node)
        {
            if (node.Left.Arc != null)
                return node;
            else if (node.Rank == node.Left.Rank)
            {
                // Perform an AA Skew
                var left = node.Left;
                var leftRight = left.Right;
                node.Left = leftRight;
                left.Right = node;

                if (node.Parent != null)
                {
                    if (node.Parent.Left == node)
                        node.Parent.Left = left;
                    else
                        node.Parent.Right = left;
                }
                else
                    Root = left;

                left.Parent = node.Parent;
                node.Parent = left;
                leftRight.Parent = node;      

                return left;
            }
            else
                return node;
        }

        public VoronoiArcObject FindArcAbove(Vector2d point, double yLine)
        {
            TreeNode node = Root;

            // Go down until we hit an arc node
            while (node.Arc == null)
            {
                var sep = node.Separator;
                var site1 = graph.Faces[sep.Face].Location;
                var site2 = graph.Faces[graph.Edges[sep.Opposite].Face].Location;
                var intersection = Line.ComputeEquidistantPoint(site1, site2, yLine);

                if (intersection.X >= point.X)
                    node = node.Left;
                else
                    node = node.Right;
            }

            return node.Arc;
        }
    }
}
