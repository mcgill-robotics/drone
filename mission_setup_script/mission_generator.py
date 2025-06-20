import json
import os
from pathlib import Path
import sys


class MinimalPublisher:

    def make_waypoint_dict(self, latitude, longitude, altitude):
        return {
            "AMSLAltAboveTerrain": None,
            "Altitude": 50,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,
            "doJumpId": 14,
            "frame": 3,
            "params": [0, 0, 0, None, latitude, longitude, altitude],
            "type": "SimpleItem"
        }

    def __init__(self, runway_number):
        path = os.path.join(os.path.dirname(__file__),
                            f"comp_immutables_runway_{runway_number}.json")
        with open(path, "r") as fp:
            self.immutables_json = json.load(fp)
        path = os.path.join(os.path.dirname(__file__), f"comp_mutables.json")
        with open(path, "r") as fp:
            self.mutables_json = json.load(fp)

        mapping_survey_polygon = []
        geofence_polygon = []
        mapping_survey_object = {
            "TransectStyleComplexItem": {
                "CameraCalc": {
                    "AdjustedFootprintFrontal": 11.785996055226825,
                    "AdjustedFootprintSide": 15.725000000000001,
                    "CameraName": "Custom Camera",
                    "DistanceMode": 1,
                    "DistanceToSurface": 50,
                    "FixedOrientation": False,
                    "FocalLength": 6,
                    "FrontalOverlap": 70,
                    "ImageDensity": 1.2923241288625904,
                    "ImageHeight": 3040,
                    "ImageWidth": 4056,
                    "Landscape": True,
                    "MinTriggerInterval": 0,
                    "SensorHeight": 4.712,
                    "SensorWidth": 6.287,
                    "SideOverlap": 70,
                    "ValueSetIsDistance": True,
                    "version": 2
                },
                "CameraShots": 21,
                "CameraTriggerInTurnAround": False,
                "HoverAndCapture": True,
                "Items": [],
                "Refly90Degrees": False,
                "TurnAroundDistance": 5,
                "VisualTransectPoints": mapping_survey_polygon,
                "version": 2
            },
            "angle": 0,
            "complexItemType": "survey",
            "entryLocation": 0,
            "flyAlternateTransects": False,
            "polygon": mapping_survey_polygon,
            "type": "ComplexItem",
            "splitConcavePolygons": False,
            "version": 5
        }

        payload_survey_polygon = []
        payload_survey_object = {
            "TransectStyleComplexItem": {
                "CameraCalc": {
                    "AdjustedFootprintFrontal": 11.785996055226825,
                    "AdjustedFootprintSide": 15.725000000000001,
                    "CameraName": "Custom Camera",
                    "DistanceMode": 1,
                    "DistanceToSurface": 50,
                    "FixedOrientation": False,
                    "FocalLength": 6,
                    "FrontalOverlap": 70,
                    "ImageDensity": 1.2923241288625904,
                    "ImageHeight": 3040,
                    "ImageWidth": 4056,
                    "Landscape": True,
                    "MinTriggerInterval": 0,
                    "SensorHeight": 4.712,
                    "SensorWidth": 6.287,
                    "SideOverlap": 70,
                    "ValueSetIsDistance": True,
                    "version": 2
                },
                "CameraShots": 21,
                "CameraTriggerInTurnAround": False,
                "HoverAndCapture": True,
                "Items": [],
                "Refly90Degrees": False,
                "TurnAroundDistance": 5,
                "VisualTransectPoints": payload_survey_polygon,
                "version": 2
            },
            "angle": 0,
            "complexItemType": "survey",
            "entryLocation": 0,
            "flyAlternateTransects": False,
            "polygon": payload_survey_polygon,
            "type": "ComplexItem",
            "splitConcavePolygons": False,
            "version": 5
        }
        objects = []

        self.plan_dict = {
            "fileType": "Plan",
            "geoFence": {
                "circles": [],
                "polygons": [{
                    "inclusion": True,
                    "polygon": geofence_polygon,
                    "version": 1
                }],
                "version":
                2
            },
            "groundStation": "QGroundControl",
            "mission": {
                "cruiseSpeed": 15,
                "firmwareType": 12,
                "globalPlanAltitudeMode": 1,
                "hoverSpeed": 5,
                "items": objects,
                "plannedHomePosition": [47.3977419, 8.545594, 487.989],
                "vehicleType": 2,
                "version": 2
            },
            "rallyPoints": {
                "points": [],
                "version": 2
            },
            "version": 1
        }

        # Boundary
        for entry in self.immutables_json["boundary"]:
            geofence_polygon.append([entry["latitude"], entry["longitude"]])

        # Takeoff
        # takeoff = {
        #     "AMSLAltAboveTerrain": None,
        #     "Altitude": 50,
        #     "AltitudeMode": 1,
        #     "autoContinue": True,
        #     "command": 24,
        #     "doJumpId": 1,
        #     "frame": 3,
        #     "params": [0, 0, None, None, None, None, 50],
        #     "type": "SimpleItem"
        # }
        # objects.append(takeoff)

        # lap
        for entry in self.mutables_json["lap"]:
            objects.append(
                self.make_waypoint_dict(entry["latitude"], entry["longitude"],
                                        entry["altitude"]))

        # survey zones

        for entry in self.immutables_json["mapping_survey_area"]:
            mapping_survey_polygon.append(
                [entry["latitude"], entry["longitude"]])

        for entry in self.immutables_json["payload_survey_area"]:
            payload_survey_polygon.append(
                [entry["latitude"], entry["longitude"]])

        objects.append(mapping_survey_object)
        objects.append(payload_survey_object)

        # RTL
        # rtl = {
        #     "AMSLAltAboveTerrain": None,
        #     "Altitude": 50,
        #     "AltitudeMode": 1,
        #     "autoContinue": True,
        #     "command": 20,
        #     "doJumpId": 1,
        #     "frame": 3,
        #     "params": [None, None, None, None, None, None, None],
        #     "type": "SimpleItem"
        # }
        # objects.append(rtl)
        #
        # # Landing
        # landing = {
        #     "AMSLAltAboveTerrain": None,
        #     "Altitude": 50,
        #     "AltitudeMode": 1,
        #     "autoContinue": True,
        #     "command": 23,
        #     "doJumpId": 1,
        #     "frame": 3,
        #     "params": [0, 0.5, 0, 0, 0, 0, 0.],
        #     "type": "SimpleItem"
        # }
        # objects.append(landing)
        #
        # Remember to publish to /start topic to start the mission
        with open(
                os.path.join(os.path.dirname(__file__), "generated_plan.plan"),
                "w") as fp:
            json.dump(self.plan_dict, fp)


def main(runway):
    minimal_publisher = MinimalPublisher(runway)


if __name__ == '__main__':
    if (len(sys.argv) != 2 or int(sys.argv[1]) not in [1, 2]):
        print(
            f"Incorrect use of the script: Provide argument 1 or 2 for the runway the mission will take place in"
        )
        exit()

    main(int(sys.argv[1]))
