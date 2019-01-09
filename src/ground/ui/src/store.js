import { createStore, applyMiddleware, combineReducers } from 'redux';
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
  collapsed: true
});

const middleware = applyMiddleware(logger, communicator);

export default createStore(reducers, middleware);
