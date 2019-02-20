package org.team1540.robot2019.datastructures;

import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.ManualFastSearchTree;
import org.team1540.robot2019.datastructures.utils.ManualFastSearchTree.Traversal;

/**
 * Coordinate frame based transform management
 *
 * Currently supports only one tree of transforms at a time. Each sent transform must be from a direct parent to child. Transform queries can be between any coordinate frames.
 */
public class TransformManager {

    private Map<String, CoordinateFrame> frameMap = new HashMap<>(); // Frame Name -> CoordinateFrame
    private Map<Integer, ManualFastSearchTree<CoordinateFrame>> frameTreeMap = new HashMap<>(); // TreeID -> Tree
    private int treeIDInc = 0;

    /**
     * @param transform TransformStamped to be sent, must be directly from parent to child
     */
    public void sendTransform(Transform3DStamped transform) {
        /*
        if fromFrame does exist
            if toFrame does exist
                if fromFrame is direct parent of toFrame
                    update
                    return
                else - merge trees
                    assert toFrame is a root
                    merge trees
                    update
                    return
            else - toFrame doesn't exist
                create toFrame
                update
                return
        else - fromFrame doesn't exist
            if toFrame does exist
                assert toFrame is a root (doesn't have a parent)
                fromFrame is new root, child is toFrame
                update
                return
            else - toFrame doesn't exist
                Make a new searchtree with the new data
                update
                return
         */

        String fromId = transform.getFromId();
        String toId = transform.getToId();

        CoordinateFrame fromFrame = frameMap.get(fromId);
        CoordinateFrame toFrame = frameMap.get(toId);

        ManualFastSearchTree<CoordinateFrame> toTree = toFrame == null ? null : frameTreeMap.get(toFrame.getTreeID());
        ManualFastSearchTree<CoordinateFrame> fromTree = fromFrame == null ? null : frameTreeMap.get(fromFrame.getTreeID());

        if (fromFrame != null) {
            if (toFrame != null) { // fromFrame and toFrame both exist
                if (toTree.getParent(toFrame).equals(fromFrame)) {
                    // good to go
                } else {
                    if (!toTree.getRoot().equals(toFrame)) {
                        throw new IllegalArgumentException("Destination frame " + toFrame + " is not a direct child of " + fromFrame + " and already has a parent.");
                    }
                    int toTreeID = toFrame.getTreeID();
                    int fromTreeID = fromFrame.getTreeID();
                    fromTree.add(toTree, fromFrame);
                    toTree.getValues().forEach(value -> {
                        value.setTreeID(fromTreeID);
                    });
                    frameTreeMap.remove(toTreeID);
                }
            } else { // fromFrame does exist, toFrame does not exist
                toFrame = new CoordinateFrame(toId, fromFrame.getTreeID());
                frameMap.put(toId, toFrame);
                fromTree.add(toFrame, fromFrame);
            }
        } else {
            if (toFrame != null) { // fromFrame does not exist, toFrame does
                if (!toTree.getRoot().equals(toFrame)) {
                    throw new IllegalArgumentException("Destination frame " + toFrame + " already has a parent.");
                }
                fromFrame = new CoordinateFrame(fromId, toFrame.getTreeID());
                frameMap.put(fromId, fromFrame);
                toTree.setRoot(fromFrame);
            } else { // fromFrame and toFrame each do not exist
                treeIDInc++;
                fromFrame = new CoordinateFrame(fromId, treeIDInc);
                toFrame = new CoordinateFrame(toId, treeIDInc);
                frameMap.put(fromId, fromFrame);
                frameMap.put(toId, toFrame);
                ManualFastSearchTree<CoordinateFrame> newTree = new ManualFastSearchTree<>(fromFrame);
                frameTreeMap.put(treeIDInc, newTree);
                newTree.add(toFrame, fromFrame);
            }
        }
        toFrame.setRelativeTransform(transform.getTransform3D());
    }

    public Transform3D getTransform(String sourceId, String destId) {
        CoordinateFrame sourceFrame = frameMap.get(sourceId);
        CoordinateFrame destFrame = frameMap.get(destId);
        if (sourceFrame == null) {
            throw new NoSuchElementException("Frame " + sourceId + " is not defined.");
        }
        if (destFrame == null) {
            throw new NoSuchElementException("Frame " + destId + " is not defined.");
        }
        if (sourceFrame.getTreeID() != destFrame.getTreeID()) {
            throw new IllegalArgumentException("No path from " + sourceId + " to " + destId + ".");
        }
        Traversal shortestPath = frameTreeMap.get(sourceFrame.getTreeID()).shortestPath(sourceFrame, destFrame);
        Transform3D sum = Transform3D.IDENTITY;
        for (Object frame : shortestPath.getDownList()) {
            Transform3D relativeTransform3D = ((CoordinateFrame) frame).getRelativeTransform3D();
            if (relativeTransform3D == null) {
                continue;
            }
            sum = sum.add(relativeTransform3D);
        }
        ;
        for (Object frame : shortestPath.getUpList()) {
            Transform3D relativeTransform3D = ((CoordinateFrame) frame).getRelativeTransform3D();
            if (relativeTransform3D == null) {
                continue;
            }
            sum = sum.subtract(relativeTransform3D);
        }
        ;
        return sum;
    }
}
