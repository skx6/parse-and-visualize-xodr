
import numpy as np
from lxml import etree

from opendriveparser.elements.openDrive import OpenDrive
from opendriveparser.elements.road import Road
from opendriveparser.elements.roadLink import Predecessor as RoadLinkPredecessor, Successor as RoadLinkSuccessor, Neighbor as RoadLinkNeighbor
from opendriveparser.elements.roadType import Type as RoadType, Speed as RoadTypeSpeed
from opendriveparser.elements.roadElevationProfile import Elevation as RoadElevationProfileElevation
from opendriveparser.elements.roadLateralProfile import Superelevation as RoadLateralProfileSuperelevation, Crossfall as RoadLateralProfileCrossfall, Shape as RoadLateralProfileShape
from opendriveparser.elements.roadLanes import LaneOffset as RoadLanesLaneOffset, Lane as RoadLaneSectionLane, LaneSection as RoadLanesSection, LaneWidth as RoadLaneSectionLaneWidth, LaneBorder as RoadLaneSectionLaneBorder
from opendriveparser.elements.junction import Junction, Connection as JunctionConnection, LaneLink as JunctionConnectionLaneLink



def parse_opendrive(rootNode):
    """ Tries to parse XML tree, return OpenDRIVE object """

    # Only accept xml element
    if not etree.iselement(rootNode):
        raise TypeError("Argument rootNode is not a xml element")


    newOpenDrive = OpenDrive()

    # Header
    header = rootNode.find("header")

    if header is not None:

        # Reference
        if header.find("geoReference") is not None:
            pass

    # Junctions
    for junction in rootNode.findall("junction"):

        newJunction = Junction()

        newJunction.id = int(junction.get("id"))
        newJunction.name = str(junction.get("name"))

        for connection in junction.findall("connection"):

            newConnection = JunctionConnection()

            newConnection.id = connection.get("id")
            newConnection.incomingRoad = connection.get("incomingRoad")
            newConnection.connectingRoad = connection.get("connectingRoad")
            newConnection.contactPoint = connection.get("contactPoint")

            for laneLink in connection.findall("laneLink"):

                newLaneLink = JunctionConnectionLaneLink()

                newLaneLink.fromId = laneLink.get("from")
                newLaneLink.toId = laneLink.get("to")

                newConnection.addLaneLink(newLaneLink)

            newJunction.addConnection(newConnection)

        newOpenDrive.junctions.append(newJunction)



    # Load roads
    for road in rootNode.findall("road"):

        newRoad = Road()

        newRoad.id = int(road.get("id"))
        newRoad.name = road.get("name")
        newRoad.junction = int(road.get("junction")) if road.get("junction") != "-1" else None

        # TODO: Problems!!!!
        newRoad.length = float(road.get("length"))

        # Links
        if road.find("link") is not None:

            predecessor = road.find("link").find("predecessor")

            if predecessor is not None:

                newPredecessor = RoadLinkPredecessor()

                newPredecessor.elementType = predecessor.get("elementType")
                newPredecessor.elementId = predecessor.get("elementId")
                newPredecessor.contactPoint = predecessor.get("contactPoint")

                newRoad.link.predecessor = newPredecessor


            successor = road.find("link").find("successor")

            if successor is not None:

                newSuccessor = RoadLinkSuccessor()

                newSuccessor.elementType = successor.get("elementType")
                newSuccessor.elementId = successor.get("elementId")
                newSuccessor.contactPoint = successor.get("contactPoint")

                newRoad.link.successor = newSuccessor

            for neighbor in road.find("link").findall("neighbor"):

                newNeighbor = RoadLinkNeighbor()

                newNeighbor.side = neighbor.get("side")
                newNeighbor.elementId = neighbor.get("elementId")
                newNeighbor.direction = neighbor.get("direction")

                newRoad.link.neighbors.append(newNeighbor)


        # Type
        for roadType in road.findall("type"):

            newType = RoadType()

            newType.sPos = roadType.get("s")
            newType.type = roadType.get("type")

            if roadType.find("speed"):

                newSpeed = RoadTypeSpeed()

                newSpeed.max = roadType.find("speed").get("max")
                newSpeed.unit = roadType.find("speed").get("unit")

                newType.speed = newSpeed

            newRoad.types.append(newType)


        # Plan view
        for geometry in road.find("planView").findall("geometry"):

            startCoord = [float(geometry.get("x")), float(geometry.get("y"))]

            if geometry.find("line") is not None:
                newRoad.planView.addLine(startCoord, float(geometry.get("hdg")), float(geometry.get("length")))

            elif geometry.find("spiral") is not None:
                newRoad.planView.addSpiral(startCoord, float(geometry.get("hdg")), float(geometry.get("length")), float(geometry.find("spiral").get("curvStart")), float(geometry.find("spiral").get("curvEnd")))

            elif geometry.find("arc") is not None:
                newRoad.planView.addArc(startCoord, float(geometry.get("hdg")), float(geometry.get("length")), float(geometry.find("arc").get("curvature")))

            elif geometry.find("poly3") is not None:
                raise NotImplementedError()

            elif geometry.find("paramPoly3") is not None:
                if geometry.find("paramPoly3").get("pRange"):

                    if geometry.find("paramPoly3").get("pRange") == "arcLength":
                        pMax = float(geometry.get("length"))
                    else:
                        pMax = None
                else:
                    pMax = None

                newRoad.planView.addParamPoly3( \
                    startCoord, \
                    float(geometry.get("hdg")), \
                    float(geometry.get("length")), \
                    float(geometry.find("paramPoly3").get("aU")), \
                    float(geometry.find("paramPoly3").get("bU")), \
                    float(geometry.find("paramPoly3").get("cU")), \
                    float(geometry.find("paramPoly3").get("dU")), \
                    float(geometry.find("paramPoly3").get("aV")), \
                    float(geometry.find("paramPoly3").get("bV")), \
                    float(geometry.find("paramPoly3").get("cV")), \
                    float(geometry.find("paramPoly3").get("dV")), \
                    pMax \
                )

            else:
                raise Exception("invalid xml")


        # Elevation profile
        if road.find("elevationProfile") is not None:

            for elevation in road.find("elevationProfile").findall("elevation"):

                newElevation = RoadElevationProfileElevation()

                newElevation.sPos = elevation.get("s")
                newElevation.a = elevation.get("a")
                newElevation.b = elevation.get("b")
                newElevation.c = elevation.get("c")
                newElevation.d = elevation.get("d")

                newRoad.elevationProfile.elevations.append(newElevation)


        # Lateral profile
        if road.find("lateralProfile") is not None:

            for superelevation in road.find("lateralProfile").findall("superelevation"):

                newSuperelevation = RoadLateralProfileSuperelevation()

                newSuperelevation.sPos = superelevation.get("s")
                newSuperelevation.a = superelevation.get("a")
                newSuperelevation.b = superelevation.get("b")
                newSuperelevation.c = superelevation.get("c")
                newSuperelevation.d = superelevation.get("d")

                newRoad.lateralProfile.superelevations.append(newSuperelevation)

            for crossfall in road.find("lateralProfile").findall("crossfall"):

                newCrossfall = RoadLateralProfileCrossfall()

                newCrossfall.side = crossfall.get("side")
                newCrossfall.sPos = crossfall.get("s")
                newCrossfall.a = crossfall.get("a")
                newCrossfall.b = crossfall.get("b")
                newCrossfall.c = crossfall.get("c")
                newCrossfall.d = crossfall.get("d")

                newRoad.lateralProfile.crossfalls.append(newCrossfall)

            for shape in road.find("lateralProfile").findall("shape"):

                newShape = RoadLateralProfileShape()

                newShape.sPos = shape.get("s")
                newShape.t = shape.get("t")
                newShape.a = shape.get("a")
                newShape.b = shape.get("b")
                newShape.c = shape.get("c")
                newShape.d = shape.get("d")

                newRoad.lateralProfile.shapes.append(newShape)


        # Lanes
        lanes = road.find("lanes")

        if lanes is None:
            raise Exception("Road must have lanes element")

        # Lane offset
        for laneOffset in lanes.findall("laneOffset"):

            newLaneOffset = RoadLanesLaneOffset()

            newLaneOffset.sPos = laneOffset.get("s")
            newLaneOffset.a = laneOffset.get("a")
            newLaneOffset.b = laneOffset.get("b")
            newLaneOffset.c = laneOffset.get("c")
            newLaneOffset.d = laneOffset.get("d")

            newRoad.lanes.laneOffsets.append(newLaneOffset)


        # Lane sections
        for laneSectionIdx, laneSection in enumerate(road.find("lanes").findall("laneSection")):

            newLaneSection = RoadLanesSection()

            # Manually enumerate lane sections for referencing purposes
            newLaneSection.idx = laneSectionIdx

            newLaneSection.sPos = laneSection.get("s")
            newLaneSection.singleSide = laneSection.get("singleSide")

            sides = dict(
                left=newLaneSection.leftLanes,
                center=newLaneSection.centerLanes,
                right=newLaneSection.rightLanes
                )

            for sideTag, newSideLanes in sides.items():

                side = laneSection.find(sideTag)

                # It is possible one side is not present
                if side is None:
                    continue

                for lane in side.findall("lane"):

                    newLane = RoadLaneSectionLane()

                    newLane.id = lane.get("id")
                    newLane.type = lane.get("type")
                    newLane.level = lane.get("level")

                    # Lane Links
                    if lane.find("link") is not None:

                        if lane.find("link").find("predecessor") is not None:
                            newLane.link.predecessorId = lane.find("link").find("predecessor").get("id")

                        if lane.find("link").find("successor") is not None:
                            newLane.link.successorId = lane.find("link").find("successor").get("id")

                    # Width
                    for widthIdx, width in enumerate(lane.findall("width")):

                        newWidth = RoadLaneSectionLaneWidth()

                        newWidth.idx = widthIdx
                        newWidth.sOffset = width.get("sOffset")
                        newWidth.a = width.get("a")
                        newWidth.b = width.get("b")
                        newWidth.c = width.get("c")
                        newWidth.d = width.get("d")

                        newLane.widths.append(newWidth)

                    # Border
                    for borderIdx, border in enumerate(lane.findall("border")):

                        newBorder = RoadLaneSectionLaneBorder()

                        newBorder.idx = borderIdx
                        newBorder.sPos = border.get("sOffset")
                        newBorder.a = border.get("a")
                        newBorder.b = border.get("b")
                        newBorder.c = border.get("c")
                        newBorder.d = border.get("d")

                        newLane.borders.append(newBorder)

                    # Road Marks
                    # TODO

                    # Material
                    # TODO

                    # Visiblility
                    # TODO

                    # Speed
                    # TODO

                    # Access
                    # TODO

                    # Lane Height
                    # TODO

                    # Rules
                    # TODO

                    newSideLanes.append(newLane)

            newRoad.lanes.laneSections.append(newLaneSection)


        # OpenDrive does not provide lane section lengths by itself, calculate them by ourselves
        for laneSection in newRoad.lanes.laneSections:

            # Last lane section in road
            if laneSection.idx + 1 >= len(newRoad.lanes.laneSections):
                laneSection.length = newRoad.planView.getLength() - laneSection.sPos

            # All but the last lane section end at the succeeding one
            else:
                laneSection.length = newRoad.lanes.laneSections[laneSection.idx + 1].sPos - laneSection.sPos

        # OpenDrive does not provide lane width lengths by itself, calculate them by ourselves
        for laneSection in newRoad.lanes.laneSections:
            for lane in laneSection.allLanes:
                widthsPoses = np.array([x.sOffset for x in lane.widths] + [laneSection.length])
                widthsLengths = widthsPoses[1:] - widthsPoses[:-1]
                
                for widthIdx, width in enumerate(lane.widths):
                    width.length = widthsLengths[widthIdx]

        # Objects
        # TODO

        # Signals
        # TODO

        newOpenDrive.roads.append(newRoad)

    return newOpenDrive
