package org.team1540.robot2019.datastructures.utils;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.NoSuchElementException;

/**
 * A tree with manually-defined structure suited for pathing between elements. Provides some of the functionality of ROS's TF.
 *
 * @author {edelmanjm}
 */
public class ManualFastSearchTree<T> {

    private HashMap<T, Node<T>> nodes = new HashMap<>();
    private Node<T> root;

    /**
     * Create a new tree.
     *
     * @param rootVal The value of the root node.
     */
    public ManualFastSearchTree(T rootVal) {
        this.root = new Node<>(rootVal, null);
        nodes.put(rootVal, root);
    }

    /**
     * Adds a value with the specified parent node.
     *
     * @param val The value of the new node.
     * @param parent The value of the parent node for the new node.
     */
    public void add(T val, T parent) {
        Node<T> parentNode = nodes.get(parent);
        if (parentNode == null) {
            throw new NoSuchElementException("Parent " + parent + " not in tree.");
        }
        if (nodes.get(val) != null) {
            throw new IllegalStateException("Val " + val + " already in tree.");
        }
        Node<T> newNode = new Node<>(val, parentNode);
        parentNode.addChild(newNode);
        nodes.put(val, newNode);
    }

    /**
     * Adds a tree to the tree with the specified parent node.
     *
     * @param tree The tree to add.
     * @param parent The value of the parent node for the tree.
     */
    public void add(ManualFastSearchTree<T> tree, T parent) {
        Node<T> parentNode = nodes.get(parent);
        if (parentNode == null) {
            throw new NoSuchElementException("Parent " + parent + " not in tree.");
        }
        tree.root.parent = parentNode;
        parentNode.addChild(tree.root);
        nodes.putAll(tree.nodes);
    }

    /**
     * Sets the root to a new node and makes the previous root a child of the new root.
     *
     * @param val The value of the new root.
     */
    public void setRoot(T val) {
        if (nodes.get(val) != null) {
            throw new IllegalStateException("Val " + val + " already in tree.");
        }
        Node<T> newNode = new Node<>(val, null);
        newNode.addChild(root);
        nodes.put(val, newNode);
        root.parent = newNode;
        root = newNode;
    }

    /**
     * Gets the root node's value.
     *
     * @return The value of the root node.
     */
    public T getRoot() {
        return root.val;
    }

    /**
     * Gets the parent node's value of a value.
     *
     * @param val The value.
     * @return The parent node's value.
     */
    public T getParent(T val) {
        Node<T> tNode = nodes.get(val);
        if (tNode == null) {
            throw new NoSuchElementException("Val " + val + " not in tree.");
        }
        if (tNode.getParent() == null) {
            return null;
        }
        return tNode.getParent().val;
    }

    public Collection<T> getValues() {
        return nodes.keySet();
    }

    /**
     * Finds the shortest path between two values, as defined by levels in the tree.
     *
     * @param val1 The value to move up in the tree towards.
     * @param val2 The value to move down in the tree towards.
     * @return A traversal representing the path.
     */
    public Traversal shortestPath(T val1, T val2) {

        if (val1.equals(val2)) {
            Traversal traversal = new Traversal();
            traversal.root = val1;
            return traversal;
        }

        if (nodes.get(val1) == null) {
            throw new NoSuchElementException("Val " + val1 + " not in tree.");
        }
        if (nodes.get(val2) == null) {
            throw new NoSuchElementException("Val " + val2 + " not in tree.");
        }

        LinkedList<T> val1ToRoot = new LinkedList<>();
        Node<T> current = nodes.get(val1);
        while (!current.equals(root)) {
            val1ToRoot.addLast(current.val);
            current = current.parent;
        }
        val1ToRoot.addLast(root.val);

        LinkedList<T> val2ToRoot = new LinkedList<>();
        current = nodes.get(val2);
        while (!current.equals(root)) {
            val2ToRoot.addLast(current.val);
            current = current.parent;
        }
        val2ToRoot.addLast(root.val);

        Traversal traversal = new Traversal();

        if (val1.equals(root.val)) {
            traversal.root = root.val;
            traversal.setDownList(val2ToRoot);
            return traversal;
        } else if (val2.equals(root.val)) {
            traversal.root = root.val;
            traversal.setUpList(val1ToRoot);
            return traversal;
        } else {
            Iterator<T> d1 = val1ToRoot.descendingIterator();
            Iterator<T> d2 = val2ToRoot.descendingIterator();

            T currentVal1, currentVal2;
            do {
                currentVal1 = d1.next();
                currentVal2 = d2.next();
            } while (currentVal1.equals(currentVal2));

            traversal.setRoot(nodes.get(currentVal1).parent.val);
            traversal.getUpList().addFirst(currentVal1);
            d1.forEachRemaining(traversal.getUpList()::addFirst);
            traversal.getDownList().addLast(currentVal2);
            d2.forEachRemaining(traversal.getDownList()::addLast);

            return traversal;
        }
    }

    @SuppressWarnings("unused")
    private class Node<U> {

        private U val;
        private Node<U> parent;
        private HashSet<Node<U>> children = new HashSet<>();


        private Node(U val, Node<U> parent) {
            this.val = val;
            if (parent != null) {
                this.parent = parent;
            }
        }

        private void addChild(Node<U> child) {
            children.add(child);
        }

        private Node<U> getParent() {
            return parent;
        }

        private HashSet<Node<U>> getChildren() {
            return children;
        }
    }

    /**
     * Traversals are paths through the tree. They consist of an up list, going from first node to a common root, a root, and a down list, going from the root to the second node.
     */
    @SuppressWarnings("WeakerAccess")
    public class Traversal {

        T root;
        LinkedList<T> upList = new LinkedList<>();
        LinkedList<T> downList = new LinkedList<>();

        public T getRoot() {
            return root;
        }

        public void setRoot(T root) {
            this.root = root;
        }

        public LinkedList<T> getUpList() {
            return upList;
        }

        public LinkedList<T> getDownList() {
            return downList;
        }

        private void setUpList(LinkedList<T> upList) {
            this.upList = upList;
        }

        private void setDownList(LinkedList<T> downList) {
            this.downList = downList;
        }
    }

}