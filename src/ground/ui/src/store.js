import { createStore, applyMiddleware, combineReducers } from "redux";
import { createLogger } from "redux-logger";

import communicator from "./communicator";
import telemetryReducer from "./reducers/telemetryReducer";
import settingsReducer from "./reducers/settingsReducer";

const reducers = combineReducers({
  telemetry: telemetryReducer,
  settings: settingsReducer
});

const logger = createLogger({
  predicate: (getState, action) => {
    let shouldLog = action.type === "TELEMETRY";
    return shouldLog;
  },
  collapsed: true
});

const middleware = applyMiddleware(logger, communicator);

export default createStore(reducers, middleware);
