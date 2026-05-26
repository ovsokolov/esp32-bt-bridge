# Future Enhancements

## AVRCP confirmed playback state

Current behavior is partly optimistic: when the car sends PLAY or PAUSE, the AG side updates its local playback state and notifies the car before the HF/phone side has confirmed the final AVRCP state. This keeps the car UI responsive and starts the AG audio path immediately, but it can briefly disagree with the phone.

Observed risk:
- The car screen may show PLAYING, then switch back if the phone reports PAUSED/STOPPED shortly after.
- A car with a single play/pause toggle can send the opposite command if its UI state is stale.
- Logs become harder to reason about because AG local state and phone-confirmed state can differ for a short time.

Possible improvement:
- Keep AG local audio routing fast on car PLAY/PAUSE.
- Delay the AVRCP playback status notification sent to the car until HF forwards a confirmed phone status (`AVRCP_NOTIFY_PLAYBACK` or `AVRCP_STATUS`).
- Add a short fallback timeout, around 1 second, so the car UI still updates if the phone does not send a confirmation.

Expected tradeoff:
- More stable car UI and fewer play/pause state flips.
- Slightly slower visible status updates on the car screen.
- More logic needed to track pending car commands and confirmation timeouts.

## A2DP sample-rate policy

We briefly prepared a local test to force both A2DP links to a single SBC sample rate, first 48 kHz and then 44.1 kHz, but did not validate it on hardware.

Possible improvement:
- Add an explicit sample-rate policy instead of leaving it fully implicit in Bluedroid negotiation.
- Test three modes: default negotiation, force/prefer 44.1 kHz, and force/prefer 48 kHz.
- Prefer matching both bridge links to the same rate so phone->HF, I2S, and AG->car do not run mixed clocks.

Expected tradeoff:
- 44.1 kHz is likely safest for music sources.
- 48 kHz may match some car DSP paths better.
- Hard-forcing only one rate can break compatibility if either phone or car rejects that SBC capability, so a prefer-with-fallback mode is probably better than force-only.
