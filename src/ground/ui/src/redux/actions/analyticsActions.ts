export const togglePlayback = () => ({ type: "TOGGLE_PLAYBACK" as const });

export const playback = (telemetry: any) => ({
  type: "PLAYBACK" as const,
  payload: telemetry
});

export const loadInteropData = (data: any) => ({
  type: "INTEROP_DATA" as const,
  payload: data
});

export const toggleRecording = () => ({ type: "TOGGLE_RECORD" as const });
