/*******************************************************************************
 * Author: Philip Etter
 *
 * Description: This is the balanced binary tree data structure which I use
 * to maintain the breach line of parabolas.
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
    class TreeNode
    {
        public VoronoiArcObject Arc;
        public HalfEdge Separator;
        public int Rank = 0;
        public TreeNode Left;
        public TreeNode Right;
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

    class BalancedBinaryTreeStatus : IVoronoiStatusStructure
    {
        private TreeNode Root;
        private Dictionary<VoronoiArcObject, TreeNode> LeafMap = new Dictionary<VoronoiArcObject, TreeNode>();
        private VoronoiGraph graph;
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

        public void GetNeighborhood(VoronoiArcObject obj, out VoronoiArcObject prevPrev, out VoronoiArcObject prev, out VoronoiArcObject succ,
            out VoronoiArcObject succSucc, out HalfEdge prevSplittingEdge, out HalfEdge succSplittingEdge)
        {
            var node = LeafMap[obj];
            prev = node.Left?.Arc;
            prevPrev = node.Left?.Left?.Arc;
            succ = node.Right?.Arc;
            succSucc = node.Right?.Right?.Arc;

            TreeNode leftSep;
            TreeNode rightSep;
            FindAdjacentSeparators(node, out leftSep, out rightSep);

            prevSplittingEdge = leftSep?.Separator;
            succSplittingEdge = rightSep?.Separator;
        }

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

        public void Split(VoronoiArcObject currentObj, VoronoiArcObject newObj, HalfEdge splittingEdgeCurrentSide, HalfEdge splittingEdgeNewSide)
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

        private void PropogateRankOnDelete(TreeNode startNode)
        {
            for (TreeNode current = startNode.Parent; current != null; current = current.Parent)
            {
                var newRank = Math.Max(current.Left.Rank, current.Right.Rank - 1) + 1;
                current.Rank = newRank;

                current = BalanceSplit(current);
            }
        }

        private void PropogateRankOnInsert(TreeNode startNode)
        {
            // Propogate rank difference correctly + balance tree
            for (TreeNode current = startNode.Parent; current != null; current = current.Parent)
            {
                current = BalanceSkew(current);
                current = BalanceSplit(current);
            }
        }

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
