import json
import os
from pathlib import Path


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

    def __init__(self):
        path = os.path.join(os.path.dirname(__file__), "comp_boundary.json")
        with open(path, "r") as fp:
            self.json = json.load(fp)

        survey_polygon = []
        geofence_polygon = []
        survey_object = {
            "TransectStyleComplexItem": {
                "CameraCalc": {
                    "AdjustedFootprintFrontal": 25,
                    "AdjustedFootprintSide": 10,
                    "CameraName": "Manual (no camera specs)",
                    "DistanceMode": 1,
                    "DistanceToSurface": 50,
                    "version": 2
                },
                "CameraShots": 21,
                "CameraTriggerInTurnAround": True,
                "HoverAndCapture": True,
                "Items": [],
                "Refly90Degrees": False,
                "TurnAroundDistance": 5,
                "VisualTransectPoints": survey_polygon,
                "version": 2
            },
            "angle": 0,
            "complexItemType": "survey",
            "entryLocation": 0,
            "flyAlternateTransects": False,
            "polygon": survey_polygon,
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
        for entry in self.json["boundary"]:
            geofence_polygon.append([entry["latitude"], entry["longitude"]])

        # lap
        for entry in self.json["lap"]:
            objects.append(
                self.make_waypoint_dict(entry["latitude"], entry["longitude"],
                                        entry["altitude"]))

        # survey zone
        for entry in self.json["survey_area"]:
            survey_polygon.append([entry["latitude"], entry["longitude"]])

        objects.append(survey_object)

        # Remember to publish to /start topic to start the mission
        with open(
                os.path.join(os.path.dirname(__file__), "generated_plan.plan"),
                "w") as fp:
            json.dump(self.plan_dict, fp)


def main(args=None):

    minimal_publisher = MinimalPublisher()


if __name__ == '__main__':
    main()
