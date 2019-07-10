export const transmit = (msg: string, data?: any) => ({
  type: "TRANSMIT" as const,
  payload: data !== undefined ? { msg: msg, data: data } : { msg: msg }
});

export const logReduxState = () => ({
  type: "LOG_REDUX_STATE" as const
});

export const resetReduxState = () => ({
  type: "RESET_REDUX_STATE" as const
});

export const centerOnDrone = () => ({
  type: "CENTER_ON_DRONE" as const
});
