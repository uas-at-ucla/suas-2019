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

const logger = createLogger({
  predicate: (getState, action) => {
    // let shouldLog = (action.type === 'TELEMETRY');
    let shouldLog = false;
    return shouldLog;
  },
  collapsed: true
});

const middleware = applyMiddleware(logger, communicator);

const store = createStore(reducer, middleware);

loadTimelineGrammar(store.dispatch);

export default store;
