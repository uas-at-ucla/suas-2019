import { combineReducers } from 'redux';
import { combineSelectors } from 'combine-selectors-redux';

export function combineReducersAndSelectors(reducersAndSelectors) {
  let reducers = {};
  let selectorGroups = {};
  for (let namespace in reducersAndSelectors) {
    reducers[namespace] = reducersAndSelectors[namespace].reducer;
    let selectors = reducersAndSelectors[namespace].selectors;
    if (selectors) {
      selectorGroups[namespace] = selectors;
    } 
  }

  return {
    reducer: combineReducers(reducers),
    selectors: combineSelectors(selectorGroups),
  }
}
