#pragma once
#include "Reward.h"
#include "CommonRewards.h"
#include "../Math.h"
#include "../CommonValues.h"
#include <optional>

namespace RLGC
{

    class AerialReward : public Reward
    {
    public:
        AerialReward() : in_air_reward(0), last_touch_pos(std::nullopt), lost_jump(false),
                         reward(0), dist_to_ball(), on_wall(false) {}

        virtual void Reset(const GameState &initialState) override
        {
            last_touch_pos = std::nullopt;
            lost_jump = false;
            reward = 0;
        }

        virtual float GetReward(const Player &player, const GameState &state, bool isFinal) override
        {
            // Starting dribble reward
            if (130 < state.ball.pos.z && state.ball.pos.z < 180 && player.ballTouchedStep)
            {
                float ball_speed = state.ball.vel.Length();
                if (ball_speed < 2000)
                {
                    reward += 2;
                }
            }

            if (state.ball.pos.z > 230)
            {
                if (player.ballTouchedStep)
                {
                    reward += 1;
                    last_touch_pos = state.ball.pos;

                    if (220 < state.ball.pos.z && state.ball.pos.z < 400)
                    {
                        float ball_speed = state.ball.vel.Length();
                        float ball_speed_reward = ball_speed / CommonValues::BALL_MAX_SPEED;
                        reward += ball_speed_reward;
                    }

                    if (!player.hasJumped)
                    {
                        if (!lost_jump)
                        {
                            lost_jump = true;
                        }

                        float ball_speed = state.ball.vel.Length();
                        float ball_speed_reward = ball_speed / (10 * CommonValues::BALL_MAX_SPEED);
                        reward += ball_speed_reward;
                    }
                }
                else if (player.pos.z > 300)
                {
                    Vec ballToPlayer = state.ball.pos - player.pos;
                    float dist = ballToPlayer.Length() - CommonValues::BALL_RADIUS;
                    float dist_reward = (dist * dist) / 600000.0f + 0.5f;
                    if (dist_reward < 0.6f)
                    {
                        float height_reward = state.ball.pos.z / CommonValues::CEILING_Z;
                        reward += height_reward;
                        float dist_to_ball_reward = dist_to_ball.GetReward(player, state, isFinal) * 10;
                        reward += dist_to_ball_reward;
                    }
                }
                return reward;
            }
            else
            {
                reward = 0;
                if (lost_jump)
                {
                    lost_jump = false;
                }
                return reward;
            }
        }

    private:
        int in_air_reward;
        std::optional<Vec> last_touch_pos;
        bool lost_jump;
        float reward;
        DistToBallReward dist_to_ball;
        bool on_wall;
    };
}
