import { applyMiddleware, combineReducers, createStore } from "redux";
import { createLogger } from "redux-logger";
import { createStructuredSelector } from "reselect";

import communicator from "communicator";
import loadTimelineGrammar from "protobuf/timelineGrammarUtil";

import { AppAction } from "./actions/actionTypes";

import missionReducer from "./reducers/missionReducer";
import settingsReducer from "./reducers/settingsReducer";
import telemetryReducer from "./reducers/telemetryReducer";

import missionSelector from "./selectors/missionSelector";

const reducer = combineReducers({
  settings: settingsReducer,
  telemetry: telemetryReducer,
  mission: missionReducer
});
export type AppState = ReturnType<typeof reducer>;

export const selector = createStructuredSelector({
  mission: missionSelector
});

let printingPrevState = true;
const logger = createLogger({
  predicate: (getState: Function, action: AppAction): boolean => {
    let shouldLog = action.type === "LOG_REDUX_STATE";
    // let shouldLog = false;
    return shouldLog;
  },
  stateTransformer: (state: AppState): AppState => {
    const msg = printingPrevState ? "prev derived data" : "next derived data";
    console.log(msg, selector(state));
    printingPrevState = !printingPrevState;
    return state;
  }
});

const middleware = applyMiddleware(logger, communicator);

const store = createStore(reducer, middleware);

loadTimelineGrammar(store.dispatch);

export default store;
