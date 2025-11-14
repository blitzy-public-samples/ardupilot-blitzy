/**
 * @file Synth.hpp
 * @brief Audio synthesis utilities for SITL tone alarm simulation
 * 
 * @details This header-only library provides audio synthesis capabilities for the
 *          Software-In-The-Loop (SITL) simulation environment. It generates PCM
 *          audio samples for tone alarm simulation using various waveform oscillators,
 *          ADSR envelope shaping, and multi-tone layering. The generated audio is
 *          output through the SFML audio library for realistic tone alarm testing
 *          without physical hardware.
 *          
 *          Key Features:
 *          - Multiple waveform types (sine, square, triangle, sawtooth, noise)
 *          - ADSR envelope generation (Attack, Decay, Sustain, Release)
 *          - Frequency sweeping support for chirp tones
 *          - Multi-tone stacking for complex audio signals
 *          - Configurable sample rate and bit depth (16-bit PCM)
 *          
 *          This code is adapted from the One Lone Coder synthesizer blog.
 * 
 * @note This file is only compiled when WITH_SITL_TONEALARM is defined.
 *       It is specific to SITL simulation and not used in flight hardware.
 * 
 * @see https://github.com/OneLoneCoder/synth
 * 
 * @author ArduPilot Development Team (adapted from OneLoneCoder)
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#ifndef SYNTH_HPP
#define SYNTH_HPP

////////////////////////////////////////////////////////////
// Headers
////////////////////////////////////////////////////////////
#include <SFML/Audio.hpp>
#include <cfloat>
#include <cmath>

namespace Synth
{
/**
 * @enum eWaveType
 * @brief Waveform oscillator types for audio synthesis
 * 
 * @details Defines the available waveform generation algorithms used by the
 *          oscillator function. Each waveform produces different harmonic content
 *          and timbral characteristics suitable for various tone alarm signals.
 */
enum eWaveType { 
    OSC_SINE,       ///< Pure sine wave (-1 to +1) with fundamental frequency only
    OSC_SQUARE,     ///< Square wave (-1 to +1) with odd harmonics for harsh tones
    OSC_TRIANGLE,   ///< Triangle wave (-1 to +1) with softer harmonic content
    OSC_SAW_ANA,    ///< Analog-style sawtooth (warm/slow) with band-limited harmonics
    OSC_SAW_DIG,    ///< Digital sawtooth (harsh/fast) with aliasing for efficiency
    OSC_NOISE       ///< Pseudorandom white noise for alarm effects
};

/**
 * @struct sEnvelope
 * @brief ADSR envelope parameters for amplitude modulation over time
 * 
 * @details Defines an ADSR (Attack-Decay-Sustain-Release) envelope for shaping
 *          the amplitude of generated tones. This is a standard envelope used in
 *          audio synthesis to control how a sound evolves from note-on to note-off.
 *          
 *          ADSR Phases:
 *          1. Attack: Initial ramp from 0 to dStartAmplitude over dAttackTime
 *          2. Decay: Transition from dStartAmplitude to dSustainAmplitude over dDecayTime
 *          3. Sustain: Hold at dSustainAmplitude for dSustainTime duration
 *          4. Release: Fade from dSustainAmplitude to 0 over dReleaseTime
 *          
 *          All time values are in seconds. Amplitude values are normalized (0.0 to 1.0).
 *          Total sound duration = dAttackTime + dDecayTime + dSustainTime + dReleaseTime
 * 
 * @note Envelope is applied multiplicatively to oscillator output to shape volume
 */
struct sEnvelope
{
    double dAttackTime = 0.0;           ///< Attack phase duration in seconds (0 = instant start)
    double dDecayTime = 0.0;            ///< Decay phase duration in seconds (0 = no decay)
    double dSustainTime = 1.0;          ///< Sustain phase duration in seconds (default 1.0s)
    double dReleaseTime = 0.0;          ///< Release phase duration in seconds (0 = instant cutoff)
    double dStartAmplitude = 1.0;       ///< Peak amplitude at end of attack (0.0 to 1.0)
    double dSustainAmplitude = 1.0;     ///< Sustain phase amplitude (0.0 to 1.0)
};

/**
 * @struct sTone
 * @brief Tone definition with waveform type, frequency, and amplitude
 * 
 * @details Defines a single tone component for audio synthesis. Multiple tones
 *          can be stacked (added) together to create complex audio signals with
 *          rich harmonic content. Supports frequency sweeping from dStartFrequency
 *          to dEndFrequency over the duration of the sound for chirp effects.
 *          
 *          For static tones, set dStartFrequency = dEndFrequency.
 *          For frequency sweeps (chirps), dEndFrequency != dStartFrequency creates
 *          a linear frequency transition over the sound duration.
 * 
 * @note Frequency values are in Hertz (Hz), amplitude is normalized (0.0 to 1.0)
 */
struct sTone
{
    eWaveType waveType = OSC_SINE;      ///< Waveform type for oscillator (default: sine wave)
    double dStartFrequency = 440.0;     ///< Initial frequency in Hz (default: A4 = 440Hz)
    double dEndFrequency = 440.0;       ///< Final frequency in Hz (default: no sweep)
    double dAmplitude = 1.0;            ///< Tone amplitude multiplier (0.0 to 1.0, default: full)
};

/**
 * @brief Convert frequency in Hertz to angular velocity in radians per second
 * 
 * @param[in] dHertz Frequency in Hertz (cycles per second)
 * 
 * @return Angular velocity in radians per second (ω = 2πf)
 * 
 * @note Used for oscillator phase calculation in trigonometric waveform generation
 */
double w(const double dHertz);

/**
 * @brief Multi-function oscillator generating various waveforms
 * 
 * @details Generates oscillator output for specified waveform type at given frequency
 *          and time offset. Output is normalized to range [-1.0, +1.0] for all waveforms
 *          except noise which is pseudorandom. This function implements various synthesis
 *          algorithms optimized for different timbral characteristics.
 *          
 *          Waveform Algorithms:
 *          - OSC_SINE: Pure sine wave using sin(ωt)
 *          - OSC_SQUARE: Hard-clipped sine wave for harmonic-rich tones
 *          - OSC_TRIANGLE: Inverse-sine approximation for softer harmonics
 *          - OSC_SAW_ANA: Band-limited sawtooth with 30 harmonics for warmth
 *          - OSC_SAW_DIG: Optimized sawtooth with aliasing for efficiency
 *          - OSC_NOISE: Pseudorandom white noise using rand()
 * 
 * @param[in] dHertz     Fundamental frequency in Hertz
 * @param[in] dTime      Time offset in seconds from start of waveform
 * @param[in] waveType   Waveform generation algorithm to use
 * 
 * @return Oscillator output amplitude normalized to [-1.0, +1.0] range
 * 
 * @note OSC_SAW_ANA is CPU-intensive (30 harmonics), use OSC_SAW_DIG for efficiency
 * @warning OSC_NOISE output is not deterministic and varies between calls
 * 
 * @see Based on One Lone Coder synthesizer blog at www.onelonecoder.com
 */
double osc(double dHertz, double dTime, eWaveType waveType);

/**
 * @brief Calculate ADSR envelope amplitude at specified time
 * 
 * @details Implements ADSR (Attack-Decay-Sustain-Release) envelope generation for
 *          amplitude modulation. Returns the envelope multiplier for given time offset,
 *          transitioning through attack, decay, sustain, and release phases as defined
 *          in the envelope structure.
 *          
 *          Phase Calculations:
 *          - Attack: Linear ramp from 0 to dStartAmplitude (0 to dAttackTime)
 *          - Decay: Linear transition to dSustainAmplitude (dAttackTime to dAttackTime+dDecayTime)
 *          - Sustain: Constant dSustainAmplitude (until dSustainTime expires)
 *          - Release: Linear fade to 0 (after sustain ends, over dReleaseTime)
 *          
 *          Negative amplitudes are clamped to 0.0 to prevent audio artifacts.
 * 
 * @param[in] dTime Time offset in seconds from start of envelope (0 = note on)
 * @param[in] env   Envelope structure defining ADSR parameters
 * 
 * @return Amplitude multiplier at specified time (0.0 to max envelope amplitude)
 * 
 * @note This function is called at sample rate (typically 44.1kHz or higher)
 * @warning Ensure env.dAttackTime >= DBL_EPSILON to avoid division by zero
 */
double amplitude(double dTime, sEnvelope env);

/**
 * @brief Generate multi-tone PCM audio samples with ADSR envelope
 * 
 * @details Synthesizes audio samples for multiple stacked tones with envelope shaping
 *          and writes 16-bit PCM data to SFML sound buffer. This function:
 *          1. Calculates total duration from envelope parameters
 *          2. Allocates 16-bit PCM sample buffer based on sample rate
 *          3. Generates each sample by summing all tone oscillators
 *          4. Applies frequency sweeping for each tone (start to end frequency)
 *          5. Applies ADSR envelope modulation to combined signal
 *          6. Scales by master volume and converts to 16-bit integer
 *          7. Loads samples into SFML SoundBuffer for playback
 *          
 *          Audio Format:
 *          - Bit Depth: 16-bit signed PCM (sf::Int16)
 *          - Channels: 1 (mono)
 *          - Sample Rate: Configurable (typically 44100 Hz)
 *          - Buffer Size: duration * sample_rate samples
 *          
 *          Multiple tones are additively mixed (summed) before envelope application,
 *          allowing creation of complex harmonic structures.
 * 
 * @param[out] buffer        Pointer to SFML SoundBuffer to store generated audio
 * @param[in]  env           ADSR envelope defining amplitude over time
 * @param[in]  tones         Vector of tone structures to stack (additive synthesis)
 * @param[in]  uMasterVol    Master volume scaling factor (typically 0-32767)
 * @param[in]  uSampleRate   Audio sample rate in Hz (e.g., 44100 for CD quality)
 * 
 * @return true if audio generation and buffer loading succeeded, false if buffer is null or loading failed
 * 
 * @note Allocates heap memory for sample buffer, automatically freed after loading
 * @note Frequency sweep is linear: freq(t) = start + (end-start) * (t/duration)
 * @warning High uMasterVol values can cause clipping; adjust based on number of tones
 * @warning Large uSampleRate or long envelope durations allocate significant memory
 * 
 * Source: libraries/AP_HAL_SITL/Synth.hpp:137-176
 */
bool generate(sf::SoundBuffer* buffer, sEnvelope env, std::vector<sTone> tones, unsigned uMasterVol, unsigned uSampleRate);

/**
 * @brief Generate single-tone PCM audio samples with ADSR envelope
 * 
 * @details Convenience wrapper for generating audio from a single tone. Internally
 *          creates a vector containing the single tone and calls the multi-tone
 *          generate() function. Equivalent to calling generate() with a vector
 *          containing one tone element.
 *          
 *          This overload simplifies the API for common single-tone alarm signals
 *          without requiring vector construction by the caller.
 * 
 * @param[out] buffer        Pointer to SFML SoundBuffer to store generated audio
 * @param[in]  env           ADSR envelope defining amplitude over time
 * @param[in]  tone          Single tone structure defining waveform and frequency
 * @param[in]  uMasterVol    Master volume scaling factor (typically 0-32767)
 * @param[in]  uSampleRate   Audio sample rate in Hz (e.g., 44100 for CD quality)
 * 
 * @return true if audio generation succeeded, false if failed
 * 
 * @note This is a convenience wrapper around the multi-tone generate() function
 * 
 * @see generate(sf::SoundBuffer*, sEnvelope, std::vector<sTone>, unsigned, unsigned)
 * 
 * Source: libraries/AP_HAL_SITL/Synth.hpp:192-197
 */
bool generate(sf::SoundBuffer* buffer, sEnvelope env, sTone tone, unsigned uMasterVol, unsigned uSampleRate);

////////////////////////////////////////////////////////////
// Converts frequency (Hz) to angular velocity
////////////////////////////////////////////////////////////
const double PI = 3.14159265359;  ///< Pi constant for trigonometric calculations

/**
 * @brief Convert frequency to angular velocity (implementation)
 * 
 * @details Converts frequency in Hertz to angular velocity in radians per second
 *          using the formula ω = 2πf. This is used for phase calculation in
 *          trigonometric oscillator functions.
 * 
 * @param[in] dHertz Frequency in Hertz
 * @return Angular velocity in radians per second
 */
double w(const double dHertz)
{
    return dHertz * 2.0 * PI;
}


////////////////////////////////////////////////////////////
// Multi-Function Oscillator
//
// This function was mostly "borrowed" from One Lone Coder blog at
// wwww.onelonecoder.com
//
////////////////////////////////////////////////////////////
/**
 * @brief Multi-function oscillator implementation
 * 
 * @details Generates waveforms using various synthesis algorithms. Each waveform
 *          type uses different mathematical approaches optimized for timbre and
 *          computational efficiency.
 * 
 * @param[in] dHertz    Fundamental frequency in Hz
 * @param[in] dTime     Time offset in seconds
 * @param[in] waveType  Waveform algorithm selection
 * @return Oscillator amplitude [-1.0 to +1.0]
 */
double osc(double dHertz, double dTime, eWaveType waveType)
{
    switch (waveType)
    {
    case OSC_SINE: // Pure sine wave: fundamental frequency only, no harmonics
        return sin(w(dHertz) * dTime);

    case OSC_SQUARE: // Square wave: rich odd harmonics for harsh, buzzer-like tones
        // Hard-clip sine wave to +1/-1 based on polarity
        return sin(w(dHertz) * dTime) > 0 ? 1.0 : -1.0;

    case OSC_TRIANGLE: // Triangle wave: softer harmonics than square, mellower tone
        // Use inverse-sine of sine to create triangle shape
        return asin(sin(w(dHertz) * dTime)) * (2.0 / PI);

    case OSC_SAW_ANA: // Analog-style sawtooth: warm, band-limited, CPU-intensive
    {
        double dOutput = 0.0;
        // Warmth defines harmonic count: higher = harsher (more harmonics)
        // 30 harmonics provides good balance between warmth and CPU usage
        double dWarmth = 30.0;

        // Additive synthesis: sum first 30 harmonics with 1/n amplitude falloff
        // This band-limits the sawtooth to reduce aliasing artifacts
        for (double n = 1.0; n < dWarmth; n++)
            dOutput += (sin(n * w(dHertz) * dTime)) / n;

        return dOutput * (2.0 / PI);  // Normalize amplitude
    }

    case OSC_SAW_DIG: // Digital sawtooth: harsh, aliased, computationally efficient
        // Direct sawtooth calculation using modulo for phase wrapping
        // Fast but produces aliasing at high frequencies (acceptable for alarms)
        return (2.0 / PI) * (dHertz * PI * fmod(dTime, 1.0 / dHertz) - (PI / 2.0));

    case OSC_NOISE: // White noise: pseudorandom for alarm effects and percussion
        // Generate random value in [0,1], scale to [-1,1]
        return 2.0 * ((double)rand() / (double)RAND_MAX) - 1.0;

    default:
        return 0.0;  // Invalid waveform type returns silence
    }
}

////////////////////////////////////////////////////////////
// Amplitude Modulator
////////////////////////////////////////////////////////////
/**
 * @brief ADSR envelope amplitude calculator (implementation)
 * 
 * @details Calculates envelope amplitude using piecewise linear segments for each
 *          ADSR phase. The function determines which phase the current time falls
 *          into and calculates the appropriate linear interpolation.
 *          
 *          Phase Timeline:
 *          0 -> dAttackTime: Attack phase (ramp up)
 *          dAttackTime -> dAttackTime+dDecayTime: Decay phase (ramp down)
 *          dAttackTime+dDecayTime -> dTimeOff: Sustain phase (constant)
 *          dTimeOff -> dTimeOff+dReleaseTime: Release phase (fade out)
 * 
 * @param[in] dTime Current time offset in seconds
 * @param[in] env   Envelope parameters
 * @return Amplitude multiplier [0.0 to max envelope amplitude]
 */
double amplitude(double dTime, sEnvelope env)
{
    // Calculate when sustain phase ends (start of release phase)
    double dTimeOff = env.dAttackTime + env.dDecayTime + env.dSustainTime;
    double dAmplitude = 0.0;

    // ADSR envelope evaluation (checked in reverse chronological order for efficiency)
    
    if (dTime > dTimeOff) {
        // Release phase: Linear fade from dSustainAmplitude to 0 over dReleaseTime
        // Linear interpolation: y = (x/duration) * (end - start) + start
        dAmplitude = ((dTime - dTimeOff) / env.dReleaseTime) * (0.0 - env.dSustainAmplitude) + env.dSustainAmplitude;
    }
    else if (dTime > (env.dAttackTime + env.dDecayTime)) {
        // Sustain phase: Hold constant amplitude until release
        dAmplitude = env.dSustainAmplitude;
    }
    else if (dTime > env.dAttackTime && dTime <= (env.dAttackTime + env.dDecayTime)) {
        // Decay phase: Linear transition from dStartAmplitude to dSustainAmplitude
        // Calculate time within decay phase and interpolate
        dAmplitude = ((dTime - env.dAttackTime) / env.dDecayTime) * (env.dSustainAmplitude - env.dStartAmplitude) + env.dStartAmplitude;
    }
    else if ((env.dAttackTime >= DBL_EPSILON) && dTime <= env.dAttackTime) {
        // Attack phase: Linear ramp from 0 to dStartAmplitude
        // Check for non-zero attack time to avoid division by zero
        dAmplitude = (dTime / env.dAttackTime) * env.dStartAmplitude;
    }

    // Clamp negative amplitudes to zero to prevent audio artifacts
    // (can occur due to floating-point precision in envelope calculations)
    if (dAmplitude <= 0.000)
        dAmplitude = 0.0;

    return dAmplitude;
}



////////////////////////////////////////////////////////////
/// \brief Generate sound and store in SoundBuffer
///
/// This function uses case-in
///
/// \param buffer is address to SoundBuffer where the result will be stored
/// \param envelope structure defining the ADSR Envelope
/// \param tones vector of tone structures to be stacked
/// \param master volume for the volume of sound
/// \param sample rate to set quality of sound
///
/// \return True if the sound was generate, false if it failed
///
////////////////////////////////////////////////////////////
/**
 * @brief Multi-tone PCM audio generation implementation
 * 
 * @details Synthesizes audio by iterating through each sample, generating all tones,
 *          applying envelope, and writing to 16-bit PCM buffer. The process:
 *          
 *          1. Calculate total duration and allocate sample buffer
 *          2. For each sample:
 *             a. Sum all tone oscillators (additive synthesis)
 *             b. Apply linear frequency sweep for each tone
 *             c. Calculate ADSR envelope amplitude
 *             d. Multiply signal by envelope and master volume
 *             e. Convert to 16-bit signed integer PCM
 *          3. Load samples into SFML SoundBuffer
 *          4. Clean up allocated memory
 *          
 *          Audio Format: 16-bit PCM, mono, configurable sample rate
 * 
 * @param[out] buffer       SFML SoundBuffer to receive generated audio
 * @param[in]  env          ADSR envelope parameters
 * @param[in]  tones        Vector of tones for additive synthesis
 * @param[in]  uMasterVol   Master volume scaling
 * @param[in]  uSampleRate  Sample rate in Hz
 * @return true on success, false if buffer is null or loading fails
 */
bool generate(sf::SoundBuffer* buffer, sEnvelope env, std::vector<sTone> tones, unsigned uMasterVol, unsigned uSampleRate)
{
    // Validate buffer pointer before proceeding
    if (!buffer)
        return false;

    // Calculate total sound duration from envelope parameters (in seconds)
    double dTotalDuration = env.dAttackTime + env.dDecayTime + env.dSustainTime + env.dReleaseTime;
    
    // Calculate required buffer size: duration * sample_rate = number of samples
    unsigned iBufferSize = unsigned(dTotalDuration * uSampleRate);
    
    // Allocate heap memory for 16-bit PCM samples
    sf::Int16 * iRaw;
    iRaw = new sf::Int16[iBufferSize];

    // Calculate time increment per sample (inverse of sample rate)
    double dIncrement = 1.0 / double(uSampleRate);
    double dTime = 0.0;  // Current time offset in seconds
    
    // Generate each PCM sample
    for (unsigned i = 0; i < iBufferSize; i++)
    {
        double signal = 0.0;  // Accumulated signal from all tones
            
        // Additive synthesis: Generate and sum all tone oscillators
        for (sTone t : tones)
        {
            // Calculate current frequency with linear sweep from start to end
            // Frequency interpolation: freq(t) = start + (end-start) * progress
            double dFrequency = t.dStartFrequency + ((t.dEndFrequency - t.dStartFrequency) * (double(i) / double(iBufferSize)));
            
            // Generate oscillator output and scale by tone amplitude, then add to signal
            signal = signal + (osc(dFrequency, dTime, t.waveType) * t.dAmplitude);
        }

        // Calculate ADSR envelope amplitude for current time
        double dEnvelopeAmplitude = amplitude(dTime, env);

        // Apply master volume and envelope, then convert to 16-bit PCM sample
        // Order: signal [-1,+1] * tone_amplitude * envelope * master_volume -> Int16
        *(iRaw + i) = sf::Int16(uMasterVol * signal * dEnvelopeAmplitude);

        // Advance time by one sample period
        dTime += dIncrement;
    }

    // Load PCM samples into SFML SoundBuffer (mono, 1 channel)
    bool bSuccess = buffer->loadFromSamples(iRaw, iBufferSize, 1, uSampleRate);
    
    // Free allocated PCM buffer memory
    delete[] iRaw;

    return bSuccess;
}

////////////////////////////////////////////////////////////
/// \brief Generate sound and store in SoundBuffer
///
/// This function uses case-in
///
/// \param buffer is address to SoundBuffer where the result will be stored
/// \param envelope structure defining the ADSR Envelope
/// \param tone structure for simple tone definition
/// \param master volume for the volume of sound
/// \param sample rate to set quality of sound
///
/// \return True if the sound was generate, false if it failed
///
////////////////////////////////////////////////////////////
/**
 * @brief Single-tone audio generation convenience wrapper (implementation)
 * 
 * @details Wraps single tone in vector and delegates to multi-tone generate().
 *          This simplifies the API for common single-tone alarm signals without
 *          requiring the caller to construct a vector explicitly.
 * 
 * @param[out] buffer       SFML SoundBuffer to receive generated audio
 * @param[in]  env          ADSR envelope parameters
 * @param[in]  tone         Single tone definition
 * @param[in]  uMasterVol   Master volume scaling
 * @param[in]  uSampleRate  Sample rate in Hz
 * @return true on success, false on failure
 */
bool generate(sf::SoundBuffer* buffer, sEnvelope env, sTone tone, unsigned uMasterVol, unsigned uSampleRate)
{
    // Wrap single tone in vector for multi-tone generator
    std::vector<sTone> tones;
    tones.push_back(tone);
    
    // Delegate to multi-tone implementation
    return generate(buffer, env, tones, uMasterVol, uSampleRate);
}

}
#endif  // SYNTH_HPP
