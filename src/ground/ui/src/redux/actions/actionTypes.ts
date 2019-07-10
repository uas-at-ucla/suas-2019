import * as genericActions from "./genericActions";
import * as externalActions from "./externalActions";
import * as analyticsActions from "./analyticsActions";
import * as droneActions from "./droneActions";
import * as missionActions from "./missionActions";
import * as settingsActions from "./settingsActions";

const allActions = {
  ...genericActions,
  ...externalActions,
  ...analyticsActions,
  ...droneActions,
  ...missionActions,
  ...settingsActions
};

const allActionsArray = Object.values(allActions);

export type AppAction = ReturnType<typeof allActionsArray[number]>;
