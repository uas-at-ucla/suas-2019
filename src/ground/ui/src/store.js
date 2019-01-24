import { createStore, applyMiddleware } from 'redux';
import { combineReducersAndSelectors } from './utils/reduxUtils';
import { createLogger } from 'redux-logger';

import communicator from './communicator';
import loadTimelineGrammar from './protobuf/timelineGrammarUtil';

import telemetryReducer from './reducers/telemetryReducer';
import missionReducer from './reducers/missionReducer';

export const { reducer, selector } = combineReducersAndSelectors({
  telemetry: telemetryReducer,
  missionPlan: missionReducer
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
