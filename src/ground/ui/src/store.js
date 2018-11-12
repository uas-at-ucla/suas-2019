import { createStore, applyMiddleware } from 'redux';
import { combineReducersAndSelectors } from './utils/reduxUtils';
import { createLogger } from 'redux-logger';

import communicator from './communicator';
import telemetryReducer from './reducers/telemetryReducer';
import missionReducer from './reducers/missionReducer';
import loadGroundLanguage from './protobuf/loadGroundLanguage';

const reducerAndSelectors = combineReducersAndSelectors({
  telemetry: telemetryReducer,
  missionPlan: missionReducer
});
export const selectors = reducerAndSelectors.selectors;

const logger = createLogger({
  predicate: (getState, action) => {
  	let shouldLog = (action.type === 'TELEMETRY');
    return shouldLog;
  },
  collapsed: true
});

const middleware = applyMiddleware(logger, communicator);

const store = createStore(reducerAndSelectors.reducer, middleware);

loadGroundLanguage(store.dispatch);

export default store;
