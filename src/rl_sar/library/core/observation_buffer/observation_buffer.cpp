/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "observation_buffer.hpp"
#include <stdexcept>
#include <algorithm>

ObservationBuffer::ObservationBuffer()
{
}

ObservationBuffer::ObservationBuffer(int num_envs,
                                     const std::vector<int>& obs_dims,
                                     int history_length,
                                     const std::string& priority)
    : num_envs(num_envs),
      obs_dims(obs_dims),
      history_length(history_length),
      priority(priority)
{
    if (num_envs <= 0 || history_length <= 0)
    {
        throw std::invalid_argument("num_envs and history_length must be positive");
    }

    for (int dim : obs_dims)
    {
        if (dim <= 0)
        {
            throw std::invalid_argument("All observation dimensions must be positive");
        }
        num_obs += dim;
    }

    num_obs_total = num_obs;
    if (num_obs_total <= 0)
    {
        throw std::runtime_error("Invalid total observation dimension");
    }

    // Initialize buffer: [env][time][obs]
    obs_buf.resize(num_envs);
    for (int env_idx = 0; env_idx < num_envs; ++env_idx)
    {
        obs_buf[env_idx].resize(history_length);
        for (int t = 0; t < history_length; ++t)
        {
            obs_buf[env_idx][t].resize(num_obs_total, 0.0f);
        }
    }
}

void ObservationBuffer::reset(std::vector<int> reset_idxs, const std::vector<float>& new_obs)
{
    if (obs_buf.empty())
    {
        return;
    }

    // Reset observation buffer for specified environments
    for (int env_idx : reset_idxs)
    {
        if (env_idx >= 0 && env_idx < num_envs)
        {
            // Copy new observation data to all time steps
            for (int t = 0; t < history_length; ++t)
            {
                for (int i = 0; i < num_obs_total && i < static_cast<int>(new_obs.size()); ++i)
                {
                    obs_buf[env_idx][t][i] = new_obs[i];
                }
            }
        }
    }
}

void ObservationBuffer::insert(const std::vector<float>& new_obs)
{
    if (obs_buf.empty() || new_obs.size() != static_cast<size_t>(num_obs_total))
    {
        return;
    }

    // Shift historical observations forward by one position for all environments
    for (int env_idx = 0; env_idx < num_envs; ++env_idx)
    {
        // Move from back to front to avoid overwriting
        for (int t = history_length - 1; t > 0; --t)
        {
            obs_buf[env_idx][t] = obs_buf[env_idx][t - 1];
        }

        // Insert new observation at the first position
        obs_buf[env_idx][0] = new_obs;
    }
}

/**
 * @brief Gets history of observations indexed by obs_ids.
 *
 * @param obs_ids Time indices into the history buffer, where 0 is the latest
 *                observation (newest frame) and history_length - 1 is the
 *                oldest observation. These are NOT term indices.
 * @return A vector containing the concatenated observations.
 *
 * Output layout depends on priority:
 *   "time" -> [frame0_all_terms, frame1_all_terms, ...]
 *             (Time-first + Current-to-Oldest + Term-order)
 *   "term" -> [term0_all_frames, term1_all_frames, ...]
 *             (Term-first + Current-to-Oldest + Time-order)
 * In both cases frame0 = newest, frameN = oldest.
 */
std::vector<float> ObservationBuffer::get_obs_vec(std::vector<int> obs_ids)
{
    if (obs_buf.empty() || obs_ids.empty())
    {
        return std::vector<float>();
    }

    // Count valid time indices (those within the buffer depth [0, history_length)).
    // obs_ids are TIME indices, not term indices.
    int num_valid_ids = 0;
    for (int obs_id : obs_ids)
    {
        if (obs_id >= 0 && obs_id < history_length)
        {
            num_valid_ids++;
        }
    }

    if (num_valid_ids == 0)
    {
        return std::vector<float>();
    }

    // Each valid frame contributes num_obs_total values regardless of priority mode.
    int output_size_per_env = num_valid_ids * num_obs_total;

    std::vector<float> output;
    output.reserve(static_cast<size_t>(num_envs) * output_size_per_env);

    if (this->priority == "time")
    {
        // Time-major: concatenate entire frames (all terms at once).
        // Output: [frame0(ang_vel,gravity,...,actions), frame1(...), ...]
        // frame0 = newest (buf index 0), frameN = oldest.
        for (int env_idx = 0; env_idx < num_envs; ++env_idx)
        {
            for (int obs_id : obs_ids)
            {
                if (obs_id >= 0 && obs_id < history_length)
                {
                    const auto& frame = obs_buf[env_idx][obs_id];
                    output.insert(output.end(), frame.begin(), frame.end());
                }
            }
        }
    }
    else if (this->priority == "term")
    {
        // Term-major: for each term, concatenate its slice across all requested frames.
        // Output: [term0(frame0,frame1,...), term1(frame0,frame1,...), ...]
        // frame0 = newest (buf index 0), frameN = oldest.
        for (int env_idx = 0; env_idx < num_envs; ++env_idx)
        {
            int obs_offset = 0;
            for (size_t i = 0; i < obs_dims.size(); ++i)
            {
                int dim = obs_dims[i];
                for (int step : obs_ids)
                {
                    if (step >= 0 && step < history_length)
                    {
                        const auto& frame = obs_buf[env_idx][step];
                        output.insert(output.end(), frame.begin() + obs_offset, frame.begin() + obs_offset + dim);
                    }
                }
                obs_offset += dim;
            }
        }
    }

    return output;
}
