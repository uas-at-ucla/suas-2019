import { createStore, applyMiddleware } from 'redux';
import { combineReducers } from 'redux-immutable';
import { createLogger } from 'redux-logger'

import communicator from './communicator';
import telemetryReducer from './reducers/telemetryReducer';

const reducers = combineReducers({
  telemetry: telemetryReducer
});

const logger = createLogger({
  predicate: (getState, action) => {
  	let shouldLog = (action.type === 'TELEMETRY');
    return shouldLog;
  },
  stateTransformer: (state) => state.toJS(),
  collapsed: true
});

const middleware = applyMiddleware(logger, communicator);

export default createStore(reducers, middleware);
