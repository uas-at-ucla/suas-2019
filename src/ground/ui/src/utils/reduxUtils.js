import { combineReducers } from 'redux';
import { createSelector, createStructuredSelector } from 'reselect';

/* Takes an object made of reducer/selector pairs
   {
     key1: {
       reducer: someFunctionOfStateAndAction
       selector: someFunctionOfState
     },
     key2: {
       reducer: someFunctionOfStateAndAction
       selector: someFunctionOfState
     }
   }
*/
export function combineReducersAndSelectors(reducersAndSelectors) {
  let { reducers, selectors } = Object.keys(reducersAndSelectors).reduce(
    (organizedReducersAndSelectors, key) => {
      organizedReducersAndSelectors.reducers[key] = reducersAndSelectors[key].reducer;
      let selector = reducersAndSelectors[key].selector;
      if (selector) {
        organizedReducersAndSelectors.selectors[key] = selector;
      }
      return organizedReducersAndSelectors;
    },
    { reducers: {}, selectors: {} }
  );

  return {
    reducer: combineReducers(reducers),
    selector: combineSelectors(selectors),
  };
}

function combineSelectors(selectors) {
  // Inspired by https://egghead.io/lessons/react-redux-implementing-combinereducers-from-scratch
  return createStructuredSelector(Object.keys(selectors).reduce(
    (derivedData, key) => {
      derivedData[key] = createSelector(
        [state => state[key]],
        subState => selectors[key](subState)
      );
      return derivedData;
    },
    {}
  ));
}
