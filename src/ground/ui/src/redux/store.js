import { combineReducers, applyMiddleware, createStore } from 'redux';
import { createStructuredSelector } from 'reselect';
import { createLogger } from 'redux-logger';

import communicator from 'communicator';
import loadTimelineGrammar from 'protobuf/timelineGrammarUtil';

import telemetryReducer from './reducers/telemetryReducer';
import missionReducer from './reducers/missionReducer';
import settingsReducer from './reducers/settingsReducer';

import telemetrySelector from './selectors/telemetrySelector';
import missionSelector from './selectors/missionSelector';

const reducer = combineReducers({
  telemetry: telemetryReducer,
  mission: missionReducer,
  settings: settingsReducer
});

export const selector = createStructuredSelector({
  telemetry: telemetrySelector,
  mission: missionSelector
});

let printingPrevState = true;
const logger = createLogger({
  predicate: (getState, action) => {
    let shouldLog = (action.type === 'LOG_REDUX_STATE');
    // let shouldLog = false;
    return shouldLog;
  },
  stateTransformer: (state) => {
    let msg = printingPrevState ? "prev derived data" : "next derived data";
    console.log(msg, selector(state));
    printingPrevState = !printingPrevState;
    return state;
  }
});

const middleware = applyMiddleware(logger, communicator);

const store = createStore(reducer, middleware);

loadTimelineGrammar(store.dispatch);

export default store;
