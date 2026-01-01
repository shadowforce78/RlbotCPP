#pragma once
#include "Reward.h"
#include "../Math.h"
#include <cmath>
namespace RLGC {

	using namespace CommonValues;

	inline float distance2D(float x1, float z1, float x2, float z2) {
		float dx = x2 - x1;
		float dz = z2 - z1;
		return std::sqrt(dx * dx + dz * dz);
	}

	inline std::vector<float> linspace(float start_in, float end_in, int num_in) {
		std::vector<float> linspaced;

		if (num_in <= 0) {
			return linspaced;
		}
		if (num_in == 1) {
			linspaced.push_back(start_in);
			return linspaced;
		}

		float delta = (end_in - start_in) / (num_in - 1);
		for (int i = 0; i < num_in; ++i) {
			linspaced.push_back(start_in + delta * i);
		}
		return linspaced;
	}

	class MawkzyFlickReward : public Reward {
	private:
		struct PlayerState {
			bool ballControlPhase = false;
			bool airRollPhase = false;
			bool flickPhase = false;
			bool flickCompleted = false;
			Vec ballPosition = Vec(0, 0, 0);
			Vec playerForward = Vec(0, 0, 0);
			Vec playerRight = Vec(0, 0, 0);
			float airRollTime = 0.0f;
			float flickPower = 0.0f;
			float airRollDirection = 0.0f; // -1 = gauche, +1 = droite
			bool wasOnGround = false;
			bool hadFlipBefore = false;
			Vec lastBallVelocity = Vec(0, 0, 0);
		};
		std::map<uint32_t, PlayerState> playerStates;

	public:
		float ballProximityThreshold;
		float minAirRollTime;
		float minFlickPower;
		float targetFlickAngle;
		float angleTolerance;
		float ballControlReward;
		float airRollReward;
		float flickReward;
		float powerBonus;
		float precisionBonus;
		float angleBonus;
		float directionBonus;
		bool debug;

		MawkzyFlickReward(
			float ballProximityThreshold = 250.0f,
			float minAirRollTime = 0.2f,
			float minFlickPower = 800.0f,
			float targetFlickAngle = 45.0f,
			float angleTolerance = 15.0f,
			float ballControlReward = 1.0f,
			float airRollReward = 2.0f,
			float flickReward = 4.0f,
			float powerBonus = 2.0f,
			float precisionBonus = 2.0f,
			float angleBonus = 3.0f,
			float directionBonus = 2.5f,
			bool debug = false
		) : ballProximityThreshold(ballProximityThreshold), minAirRollTime(minAirRollTime),
			minFlickPower(minFlickPower), targetFlickAngle(targetFlickAngle),
			angleTolerance(angleTolerance), ballControlReward(ballControlReward),
			airRollReward(airRollReward), flickReward(flickReward),
			powerBonus(powerBonus), precisionBonus(precisionBonus),
			angleBonus(angleBonus), directionBonus(directionBonus), debug(debug) {
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				PlayerState& state = playerStates[player.carId];
				state.ballControlPhase = false;
				state.airRollPhase = false;
				state.flickPhase = false;
				state.flickCompleted = false;
				state.ballPosition = Vec(0, 0, 0);
				state.playerForward = player.rotMat.forward;
				state.playerRight = player.rotMat.right;
				state.airRollTime = 0.0f;
				state.flickPower = 0.0f;
				state.airRollDirection = 0.0f;
				state.wasOnGround = player.isOnGround;
				state.hadFlipBefore = player.HasFlipOrJump();
				state.lastBallVelocity = Vec(0, 0, 0);
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev)
				return 0.0f;

			int carId = player.carId;
			if (playerStates.find(carId) == playerStates.end()) {
				PlayerState& newState = playerStates[player.carId];
				newState.ballControlPhase = false;
				newState.airRollPhase = false;
				newState.flickPhase = false;
				newState.flickCompleted = false;
				newState.ballPosition = Vec(0, 0, 0);
				newState.playerForward = player.rotMat.forward;
				newState.playerRight = player.rotMat.right;
				newState.airRollTime = 0.0f;
				newState.flickPower = 0.0f;
				newState.airRollDirection = 0.0f;
				newState.wasOnGround = player.isOnGround;
				newState.hadFlipBefore = player.HasFlipOrJump();
				newState.lastBallVelocity = Vec(0, 0, 0);
			}

			PlayerState& st = playerStates[carId];
			float reward = 0.0f;

			Vec playerPos = player.pos;
			Vec ballPos = state.ball.pos;
			float distanceToBall = (playerPos - ballPos).Length();
			bool isOnGround = player.isOnGround;
			Vec currentForward = player.rotMat.forward;
			Vec currentRight = player.rotMat.right;

			// === PHASE 1: CONTRÔLE DE LA BALLE ===
			if (distanceToBall < ballProximityThreshold && !st.ballControlPhase) {
				st.ballControlPhase = true;
				reward += ballControlReward;

				if (debug) {
					printf("[MawkzyFlick] car_id=%d PHASE 1: Contrôle de la balle! Reward: %.2f\n",
						carId, ballControlReward);
				}
			}

			// === PHASE 2: AIR ROLL DIRECTIONNEL ===
			if (st.ballControlPhase && !isOnGround && !st.airRollPhase) {
				// Détecter l'air roll via le changement de rotation
				if (st.playerForward.Length() > 0) {
					float rotationChange = fabsf(currentForward.Dot(st.playerForward));

					if (rotationChange < 0.8f) { // Rotation significative
						st.airRollPhase = true;

						// DÉTECTER LA DIRECTION DE L'AIR ROLL
						Vec carRight = player.rotMat.right;
						float airRollDirection = carRight.z; // -1 = gauche, +1 = droite

						// Normaliser la direction
						if (abs(airRollDirection) > 0.1f) {
							st.airRollDirection = (airRollDirection > 0) ? 1.0f : -1.0f;

							if (debug) {
								printf("[MawkzyFlick] car_id=%d PHASE 2: Air roll %s détecté! Direction: %.1f\n",
									carId, (st.airRollDirection > 0) ? "DROITE" : "GAUCHE", st.airRollDirection);
							}
						}

						reward += airRollReward;
					}
				}
			}

			// === PHASE 3: FLICK 45° ===
			if (st.airRollPhase && !st.flickPhase) {
				bool hasFlipNow = player.HasFlipOrJump();

				if (st.hadFlipBefore && !hasFlipNow) {
					st.flickPhase = true;
					st.lastBallVelocity = state.prev ? state.prev->ball.vel : Vec(0, 0, 0);

					if (debug) {
						printf("[MawkzyFlick] car_id=%d PHASE 3: Flick déclenché! Direction air roll: %.1f\n",
							carId, st.airRollDirection);
					}
				}
			}

			// === PHASE 4: VÉRIFICATION COMPLÈTE ===
			if (st.flickPhase && !st.flickCompleted && player.ballTouchedStep) {
				// 1. VÉRIFIER LA PUISSANCE DU FLICK
				float ballSpeedChange = 0.0f;
				if (state.prev) {
					ballSpeedChange = (state.ball.vel - st.lastBallVelocity).Length();
					st.flickPower = ballSpeedChange;
				}

				if (ballSpeedChange > minFlickPower) {
					reward += flickReward;
					reward += (ballSpeedChange - minFlickPower) * powerBonus / 1000.0f;

					if (debug) {
						printf("[MawkzyFlick] car_id=%d Puissance flick: %.0f, Reward: %.2f\n",
							carId, ballSpeedChange, reward);
					}
				}

				// 2. VÉRIFIER L'ANGLE DU FLICK (45°)
				float flickAngle = CalculateFlickAngle(player, state);
				float angleAccuracy = CalculateAngleAccuracy(flickAngle);

				if (angleAccuracy > 0.7f) { // Angle proche de 45°
					reward += angleBonus * angleAccuracy;

					if (debug) {
						printf("[MawkzyFlick] car_id=%d Angle flick: %.1f° (cible: 45°), Accuracy: %.2f, Bonus: %.2f\n",
							carId, flickAngle, angleAccuracy, angleBonus * angleAccuracy);
					}
				}

				// 3. VÉRIFIER LA DIRECTION DU FLICK (correspond à l'air roll)
				bool directionMatches = CheckDirectionMatch(st.airRollDirection, state.ball.vel);

				if (directionMatches) {
					reward += directionBonus;

					if (debug) {
						printf("[MawkzyFlick] car_id=%d DIRECTION PARFAITE! Air roll %s ? Flick %s, Bonus: %.2f\n",
							carId,
							(st.airRollDirection > 0) ? "DROITE" : "GAUCHE",
							(st.airRollDirection > 0) ? "DROITE" : "GAUCHE",
							directionBonus);
					}
				}

				// 4. VÉRIFIER LA PRÉCISION VERS LE BUT
				Vec goalDirection = GetGoalDirection(player.team);
				Vec ballDirection = state.ball.vel.Normalized();
				float precision = ballDirection.Dot(goalDirection);

				if (precision > 0.6f) { // Précision vers le but
					reward += precisionBonus * precision;

					if (debug) {
						printf("[MawkzyFlick] car_id=%d Précision vers le but: %.2f, Bonus: %.2f\n",
							carId, precision, precisionBonus * precision);
					}
				}

				// 5. BONUS FINAL POUR MAWKZY FLICK PARFAIT
				if (angleAccuracy > 0.8f && directionMatches && precision > 0.7f) {
					float perfectBonus = 5.0f;
					reward += perfectBonus;

					if (debug) {
						printf("[MawkzyFlick] car_id=%d ?? MAWKZY FLICK PARFAIT! Bonus final: %.2f\n",
							carId, perfectBonus);
					}
				}

				st.flickCompleted = true;
			}

			// Récompense continue pour maintenir l'air roll
			if (st.airRollPhase && !isOnGround) {
				st.airRollTime += 1.0f / 120.0f;
				if (st.airRollTime > minAirRollTime) {
					reward += 0.1f; // Récompense continue
				}
			}

			// Reset si le joueur touche le sol
			if (st.wasOnGround && !isOnGround) {
				st.airRollTime = 0.0f;
			}

			// Mise à jour de l'état
			st.ballPosition = ballPos;
			st.playerForward = currentForward;
			st.playerRight = currentRight;
			st.wasOnGround = isOnGround;
			st.hadFlipBefore = player.HasFlipOrJump();

			return reward;
		}

	private:
		// Calculer l'angle du flick (0° = droit, 90° = vertical)
		float CalculateFlickAngle(const Player& player, const GameState& state) const {
			Vec carForward = Vec(player.rotMat.forward.x, player.rotMat.forward.y, 0.0f);
			Vec ballVelocity = Vec(state.ball.vel.x, state.ball.vel.y, 0.0f);

			if (carForward.Length() < 0.1f || ballVelocity.Length() < 100.0f) {
				return 0.0f;
			}

			carForward = carForward.Normalized();
			ballVelocity = ballVelocity.Normalized();

			float dotProduct = RS_MAX(-1.0f, RS_MIN(1.0f, carForward.Dot(ballVelocity)));
			float angle = acosf(dotProduct);
			return angle * 180.0f / M_PI;
		}

		// Calculer la précision de l'angle (0-1, 1 = parfait 45°)
		float CalculateAngleAccuracy(float flickAngle) const {
			float angleDiff = abs(flickAngle - targetFlickAngle);
			if (angleDiff <= angleTolerance) {
				return 1.0f - (angleDiff / angleTolerance);
			}
			return 0.0f;
		}

		// Vérifier que la direction du flick correspond à l'air roll
		bool CheckDirectionMatch(float airRollDirection, const Vec& ballVelocity) const {
			if (abs(airRollDirection) < 0.1f) return false;

			// Air roll DROITE (+1) ? Flick doit aller vers la DROITE (X positif)
			// Air roll GAUCHE (-1) ? Flick doit aller vers la GAUCHE (X négatif)
			if (airRollDirection > 0) { // Droite
				return ballVelocity.x > 200.0f; // Vitesse X positive
			}
			else { // Gauche
				return ballVelocity.x < -200.0f; // Vitesse X négative
			}
		}

		Vec GetGoalDirection(Team team) {
			if (team == Team::BLUE) {
				return (CommonValues::ORANGE_GOAL_CENTER - Vec(0, 0, 0)).Normalized();
			}
			else {
				return (CommonValues::BLUE_GOAL_CENTER - Vec(0, 0, 0)).Normalized();
			}
		}
	};




	class PressureFlickReward : public Reward {
	public:
		const float PANIC_DISTANCE = 700.0f;
		const float MIN_FLICK_SPEED = 1000.0f;
		const float TARGET_FLICK_SPEED = 2920.0f;
		const float EXPONENT = 2.5f;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.ballTouchedStep || !player.isFlipping) return 0.0f;

			if (!state.prev) return 0.0f;

			// Distance
			float dist = player.prev->pos.Dist(state.prev->ball.pos);
			if (dist > 250.0f) return 0.0f;

			// Speed Match
			float speedDiff = std::abs(player.prev->vel.Length() - state.prev->ball.vel.Length());
			if (speedDiff > 500.0f) return 0.0f;

			// Distance Check
			float closestOppDist = 100000.0f;
			for (const auto& p : state.players) {
				if (p.team != player.team) {
					float d = player.pos.Dist(p.pos);
					if (d < closestOppDist) closestOppDist = d;
				}
			}
			if (closestOppDist > PANIC_DISTANCE) return 0.0f;

			// Result Check
			bool targetOrange = player.team == Team::BLUE;
			Vec targetPos = targetOrange ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;
			Vec dirToGoal = (targetPos - state.ball.pos).Normalized();
			float velTowardsGoal = state.ball.vel.Dot(dirToGoal);

			if (velTowardsGoal > MIN_FLICK_SPEED) {
				float ratio = velTowardsGoal / TARGET_FLICK_SPEED;
				float reward = std::pow(ratio, EXPONENT);
				return std::min(reward, 2.0f);
			}

			return 0.0f;
		}
	};





	class HalfFlipReward : public Reward {
	private:
		float reward_value;
		float min_speed_gain;
		float min_facing_before;
		float min_facing_after;
		int min_ticks;
		bool debug;

		struct PlayerState {
			bool backflip_started = false;
			bool backflip_canceled = false;
			bool air_roll_detected = false;
			bool halfflip_completed = false;
			bool rewarded = false;
			float pre_flip_vel = 0.0f;
			float post_flip_vel = 0.0f;
			int flip_tick = 0;
			float facing_before = 0.0f;
			Vec initial_rotation = Vec(0, 0, 0);
			Vec rotation_at_cancel = Vec(0, 0, 0);
			Vec final_rotation = Vec(0, 0, 0);
		};

		std::unordered_map<int, PlayerState> player_states;

	public:
		HalfFlipReward(float reward_value = 8.0f,
			float min_speed_gain = 300.0f,
			float min_facing_before = -0.5f,
			float min_facing_after = 0.5f,
			int min_ticks = 4,
			bool debug = false)
			: reward_value(reward_value),
			min_speed_gain(min_speed_gain),
			min_facing_before(min_facing_before),
			min_facing_after(min_facing_after),
			min_ticks(min_ticks),
			debug(debug) {
		}

		virtual void Reset(const GameState& initial_state) override {
			player_states.clear();
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			int car_id = player.carId;
			if (player_states.find(car_id) == player_states.end()) {
				player_states[car_id] = PlayerState{};
			}

			PlayerState& st = player_states[car_id];
			float reward = 0.0f;

			// Direction vers la balle
			Vec ball_dir = state.ball.pos - player.pos;
			ball_dir = ball_dir.Normalized();

			// Direction de la voiture
			Vec forward = player.rotMat.forward;
			float facing_ball = forward.Dot(ball_dir);

			// Détection du début du backflip (saut + rotation vers l'arrière)
			if (!st.backflip_started && player.isJumping && !player.isOnGround) {
				// Vérifier si la voiture commence à se retourner (rotation pitch négative)
				Vec car_up = player.rotMat.up;
				if (car_up.z < 0.3f) { // La voiture commence à se retourner
					st.backflip_started = true;
					st.flip_tick = 0;
					st.pre_flip_vel = -forward.Dot(player.vel);
					st.facing_before = facing_ball;
					st.initial_rotation = Vec(car_up.x, car_up.y, car_up.z);

				}
			}

			// Détection du cancel du backflip
			if (st.backflip_started && !st.backflip_canceled) {
				st.flip_tick++;

				// Le cancel se caractérise par une rotation qui s'arrête
				Vec car_up = player.rotMat.up;
				float rotation_change = fabsf(car_up.z - st.initial_rotation.z);

				if (st.flip_tick >= min_ticks && rotation_change < 0.1f) {
					st.backflip_canceled = true;
					st.rotation_at_cancel = Vec(car_up.x, car_up.y, car_up.z);

				}
			}

			// Détection de l'air roll pour se retourner
			if (st.backflip_canceled && !st.air_roll_detected) {
				Vec car_up = player.rotMat.up;
				Vec car_forward = player.rotMat.forward;

				// L'air roll se caractérise par une rotation continue autour de l'axe forward
				float rotation_progress = fabsf(car_up.z - st.rotation_at_cancel.z);

				if (rotation_progress > 0.3f) { // Rotation significative détectée
					st.air_roll_detected = true;

				}
			}

			// Vérification du halfflip complet
			if (st.backflip_canceled && st.air_roll_detected && !st.halfflip_completed) {
				// Vérifier que la voiture fait face à la balle maintenant
				if (facing_ball > min_facing_after) {
					st.halfflip_completed = true;
					st.post_flip_vel = forward.Dot(player.vel);
					float speed_gain = st.post_flip_vel + st.pre_flip_vel;

					if (speed_gain > min_speed_gain) {
						reward = reward_value;

					}
				}
			}

			// Reset si au sol ou trop longtemps après le flip
			if (player.isOnGround || st.flip_tick > 50) {
				st.backflip_started = false;
				st.backflip_canceled = false;
				st.air_roll_detected = false;
				st.halfflip_completed = false;
				st.rewarded = false;
				st.flip_tick = 0;
			}

			return reward;
		}
	};

	class ShadowDefenseReward : public Reward {
	public:
		float maxBallGoalDistance;
		float maxLateralOffset;
		float idealSpacingFromBall;
		float spacingTolerance;
		ShadowDefenseReward(
			float maxBallGoalDistance = 3500.f,
			float maxLateralOffset = 1200.f,
			float idealSpacingFromBall = 1200.f,
			float spacingTolerance = 700.f
		) : maxBallGoalDistance(maxBallGoalDistance),
			maxLateralOffset(maxLateralOffset),
			idealSpacingFromBall(idealSpacingFromBall),
			spacingTolerance(spacingTolerance) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			Vec goalCenter = player.team == Team::BLUE ? CommonValues::BLUE_GOAL_CENTER : CommonValues::ORANGE_GOAL_CENTER;
			float goalDirection = player.team == Team::BLUE ? -1.f : 1.f;

			Vec goalToBall = state.ball.pos - goalCenter;
			if (goalDirection * goalToBall.y < 0.f)
				return 0.f;

			float ballDist = goalToBall.Length();
			if (ballDist > maxBallGoalDistance || ballDist < 100.f)
				return 0.f;

			Vec goalDir = goalToBall / ballDist;

			Vec goalToPlayer = player.pos - goalCenter;
			float playerDepth = goalDir.Dot(goalToPlayer);
			if (playerDepth < 0.f || playerDepth > ballDist + 250.f)
				return 0.f;

			Vec lateral = goalToPlayer - goalDir * playerDepth;
			float lateralOffset = lateral.Length();
			float lateralWeight = 1.f - RS_CLAMP(lateralOffset / maxLateralOffset, 0.f, 1.f);
			if (lateralWeight <= 0.f)
				return 0.f;

			float spacing = ballDist - playerDepth;
			float spacingWeight = 1.f - RS_CLAMP(fabsf(spacing - idealSpacingFromBall) / spacingTolerance, 0.f, 1.f);
			if (spacingWeight <= 0.f)
				return 0.f;

			Vec toBall = state.ball.pos - player.pos;
			float facingWeight = 0.f;
			if (toBall.Length() > 100.f)
				facingWeight = RS_MAX(0.f, player.rotMat.forward.Dot(toBall.Normalized()));

			float velocityWeight = 0.f;
			if (player.vel.Length() > 100.f) {
				Vec desired = toBall.Normalized();
				Vec defensiveDir = Vec(0.f, goalDirection, 0.f);
				Vec blendBase = desired * 0.6f + defensiveDir * 0.4f;
				if (blendBase.Length() < 1e-3f)
					blendBase = desired;
				Vec blended = blendBase.Normalized();
				velocityWeight = RS_MAX(0.f, player.vel.Normalized().Dot(blended));
			}

			float controlWeight = 0.4f * facingWeight + 0.6f * velocityWeight;
			return lateralWeight * spacingWeight * controlWeight;
		}
	};

	class DribbleAirdribbleBumpDemoRewardv1 : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			float reward = 0.f;
			if (state.ball.pos.y > 4600 && std::abs(state.ball.pos.x) < GOAL_WIDTH_FROM_CENTER && state.ball.vel.y > 0) { // if the ball is in front of the opponents net and going towards it
				if (player.pos.y > state.ball.pos.y && std::abs(player.pos.x) < GOAL_WIDTH_FROM_CENTER) { // if the player is in front of the ball and in front of the net
					reward += 0.001f; // With the reward being 2000 atm, this should reward it 2 per step, or 60 per second. (this might be to high)
					if (player.eventState.bump) { // if the player bumped the opponent
						reward += 0.5f * (state.ball.vel.y / 1000); // 6000 is max ball speed but will rarely be that fast. scaling by 1000 should be good because player speed averages around 1400
					}
					if (player.eventState.demo) { // if the player demoed the opponent
						reward += 1.0f * (state.ball.vel.y / 1000); // this scales it by the speed of the ball towards the net / opponents side
					}
				}
			}
			return reward;
		}
	};



	class DribbleAirdribbleBumpDemoReward : public Reward {
	private:
		std::unordered_map<uint32_t, uint64_t> lastBumpTick;
		std::unordered_map<uint32_t, uint64_t> lastDemoTick;
	public:
		float positionRewardWeight;
		float bumpRewardWeight;
		float demoRewardWeight;
		DribbleAirdribbleBumpDemoReward(
			float positionWeight = 0.001f,   // Increased from 0.001f
			float bumpWeight = 0.5f,       // Increased from 0.5f
			float demoWeight = 1.0f       // Increased from 1.0f
		) : positionRewardWeight(positionWeight),
			bumpRewardWeight(bumpWeight),
			demoRewardWeight(demoWeight) {
		}
		virtual void Reset(const GameState& initialState) override {
			lastBumpTick.clear();
			lastDemoTick.clear();
		}
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.lastArena) return 0.0f;
			float reward = 0.0f;
			Vec targetGoal = (player.team == Team::BLUE) ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;
			float goalY = targetGoal.y;
			float goalWidthHalf = CommonValues::GOAL_WIDTH_FROM_CENTER;
			float distToGoalY = std::abs(state.ball.pos.y - goalY);
			bool ballNearOpponentGoal = distToGoalY < 1500.0f;
			bool ballInGoalWidth = std::abs(state.ball.pos.x) < goalWidthHalf + 500.0f;
			Vec toGoal = targetGoal - state.ball.pos;
			float toGoalLen = toGoal.Length();
			Vec dirToGoal = (toGoalLen > 1e-6f) ? toGoal / toGoalLen : Vec(0, 0, 0);
			float ballSpeedTowardGoal = state.ball.vel.Dot(dirToGoal);
			bool ballMovingTowardGoal = ballSpeedTowardGoal > 50.0f;
			if (ballNearOpponentGoal && ballInGoalWidth && ballMovingTowardGoal) {
				float playerDistToGoal = (player.pos - targetGoal).Length();
				float ballDistToGoal = (state.ball.pos - targetGoal).Length();
				bool playerBehindBall = playerDistToGoal > ballDistToGoal - 200.0f;
				float heightDiff = std::abs(player.pos.z - state.ball.pos.z);
				bool playerAtReasonableHeight = heightDiff < 500.0f || player.pos.z < 300.0f;
				float playerDistToGoalY = std::abs(player.pos.y - goalY);
				bool playerNearOpponentGoal = playerDistToGoalY < 2000.0f;
				bool playerInAttackingZone = std::abs(player.pos.x) < goalWidthHalf + 1000.0f;
				if (playerBehindBall && playerNearOpponentGoal && playerInAttackingZone && playerAtReasonableHeight) {
					float distanceBonus = 1.0f - RS_CLAMP(playerDistToGoal / 3000.0f, 0.0f, 1.0f);
					float heightBonus = 1.0f - RS_CLAMP(heightDiff / 400.0f, 0.0f, 1.0f);
					reward += positionRewardWeight * distanceBonus * heightBonus;
					uint64_t currentTick = state.lastArena->tickCount;
					if (player.eventState.bump) {
						auto it = lastBumpTick.find(player.carId);
						if (it == lastBumpTick.end() || currentTick > it->second) {
							lastBumpTick[player.carId] = currentTick;
							float ballSpeedNorm = RS_CLAMP(ballSpeedTowardGoal / CommonValues::CAR_MAX_SPEED, 0.0f, 1.0f);
							float goalProximityBonus = 1.0f - RS_CLAMP(ballDistToGoal / 5000.0f, 0.0f, 1.0f);
							reward += bumpRewardWeight * ballSpeedNorm * (1.0f + goalProximityBonus);
						}
					}
					if (player.eventState.demo) {
						auto it2 = lastDemoTick.find(player.carId);
						if (it2 == lastDemoTick.end() || currentTick > it2->second) {
							lastDemoTick[player.carId] = currentTick;
							float ballSpeedNorm2 = RS_CLAMP(ballSpeedTowardGoal / CommonValues::CAR_MAX_SPEED, 0.0f, 1.0f);
							float goalProximityBonus2 = 1.0f - RS_CLAMP(ballDistToGoal / 5000.0f, 0.0f, 1.0f);
							reward += demoRewardWeight * ballSpeedNorm2 * (1.0f + goalProximityBonus2 * 1.5f);
						}
					}
				}
			}
			return reward;
		}
	};

	


	class Team2v2SpacingReward : public Reward {
	private:
		float optimalDistance;
		float minDistance;
		float maxDistance;
		float ballProximityWeight;
		float fieldCoverageWeight;
		float defensiveSpacingWeight;

	public:
		Team2v2SpacingReward(
			float optimalDistance = 2000.0f,
			float minDistance = 800.0f,
			float maxDistance = 4000.0f,
			float ballProximityWeight = 0.3f,
			float fieldCoverageWeight = 0.4f,
			float defensiveSpacingWeight = 0.3f
		) : optimalDistance(optimalDistance),
			minDistance(minDistance),
			maxDistance(maxDistance),
			ballProximityWeight(ballProximityWeight),
			fieldCoverageWeight(fieldCoverageWeight),
			defensiveSpacingWeight(defensiveSpacingWeight) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			const Player* teammate = nullptr;
			for (const auto& p : state.players) {
				if (p.team == player.team && p.carId != player.carId) {
					teammate = &p;
					break;
				}
			}

			if (!teammate) return 0.0f;

			float reward = 0.0f;

			float distanceToTeammate = (player.pos - teammate->pos).Length();
			float spacingReward = 0.0f;

			if (distanceToTeammate < minDistance) {
				spacingReward = -1.0f * (1.0f - distanceToTeammate / minDistance);
			}
			else if (distanceToTeammate > maxDistance) {
				spacingReward = -0.5f * ((distanceToTeammate - maxDistance) / optimalDistance);
			}
			else {
				float distanceFromOptimal = std::abs(distanceToTeammate - optimalDistance);
				spacingReward = 1.0f - (distanceFromOptimal / optimalDistance);
			}

			float playerBallDist = (player.pos - state.ball.pos).Length();
			float teammateBallDist = (teammate->pos - state.ball.pos).Length();
			float ballProximityReward = 0.0f;

			float ballDistDiff = std::abs(playerBallDist - teammateBallDist);
			if (ballDistDiff > 500.0f) {
				ballProximityReward = 0.5f;
			}
			else if (ballDistDiff < 200.0f && playerBallDist < 1000.0f && teammateBallDist < 1000.0f) {
				ballProximityReward = -0.3f;
			}

			float fieldCoverageReward = 0.0f;
			Vec fieldCenter = { 0.0f, 0.0f, 0.0f };
			Vec playerToCenter = player.pos - fieldCenter;
			Vec teammateToCenter = teammate->pos - fieldCenter;

			float angleBetween = std::abs(std::atan2(playerToCenter.y, playerToCenter.x) -
				std::atan2(teammateToCenter.y, teammateToCenter.x));
			if (angleBetween > 3.14159265f) angleBetween = 2 * 3.14159265f - angleBetween;

			if (angleBetween > 3.14159265f / 3) {
				fieldCoverageReward = std::min(1.0f, angleBetween / 3.14159265f);
			}

			float defensiveReward = 0.0f;
			Vec ownGoal = (player.team == Team::BLUE) ?
				CommonValues::BLUE_GOAL_CENTER : CommonValues::ORANGE_GOAL_CENTER;

			float playerGoalDist = (player.pos - ownGoal).Length();
			float teammateGoalDist = (teammate->pos - ownGoal).Length();

			bool ballInOpponentHalf = (player.team == Team::BLUE && state.ball.pos.y > 0) ||
				(player.team == Team::ORANGE && state.ball.pos.y < 0);

			if (ballInOpponentHalf) {
				float goalDistDiff = std::abs(playerGoalDist - teammateGoalDist);
				if (goalDistDiff > 1500.0f) {
					defensiveReward = 0.4f;
				}
			}
			else {
				if (playerGoalDist > 2000.0f && teammateGoalDist > 2000.0f) {
					defensiveReward = 0.2f;
				}
			}

			reward = spacingReward * (1.0f - ballProximityWeight - fieldCoverageWeight - defensiveSpacingWeight) +
				ballProximityReward * ballProximityWeight +
				fieldCoverageReward * fieldCoverageWeight +
				defensiveReward * defensiveSpacingWeight;

			return std::clamp(reward, -1.0f, 1.0f);
		}
	};

	class StrategicDemoReward : public Reward
	{
	public:
		// Paramètres configurables
		float goalZoneThreshold;      // Distance Y depuis le but pour considérer la "zone de but" (défaut: 2500 UU)
		float demoMultiplier;         // Multiplicateur pour les demos vs bumps (défaut: 2.0)
		float ballInZoneBonus;        // Bonus quand la balle est aussi dans la zone offensive (défaut: 0.5)
		float maxDistanceFromGoal;    // Distance max pour calculer le scaling (défaut: 5000 UU)
		float baseReward;             // Récompense de base pour un bump/demo (défaut: 0.3)
		float maxReward;              // Récompense maximale (défaut: 1.5)

		/**
		 * Constructeur avec valeurs par défaut
		 * @param goalZoneThreshold Distance Y depuis le but pour la "zone stratégique"
		 * @param demoMultiplier Multiplicateur pour les demos (vs bumps simples)
		 * @param ballInZoneBonus Bonus si la balle est en position offensive
		 * @param maxDistanceFromGoal Distance de référence pour le scaling
		 * @param baseReward Récompense de base
		 * @param maxReward Plafond de récompense
		 */
		StrategicDemoReward(
			float goalZoneThreshold = 2500.0f,
			float demoMultiplier = 2.0f,
			float ballInZoneBonus = 0.5f,
			float maxDistanceFromGoal = 5000.0f,
			float baseReward = 0.3f,
			float maxReward = 1.5f)
			: goalZoneThreshold(goalZoneThreshold),
			demoMultiplier(demoMultiplier),
			ballInZoneBonus(ballInZoneBonus),
			maxDistanceFromGoal(maxDistanceFromGoal),
			baseReward(baseReward),
			maxReward(maxReward)
		{
		}

		virtual void Reset(const GameState& initialState) override
		{
			// Pas d'état persistant à réinitialiser
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override
		{
			float reward = 0.0f;

			// Vérifier si le joueur a effectué un bump ou demo ce step
			bool didBump = player.eventState.bump;
			bool didDemo = player.eventState.demo;

			if (!didBump && !didDemo)
			{
				return 0.0f;
			}

			// Trouver la victime (adversaire qui a été bumped/demoed)
			const Player* victim = nullptr;
			for (const auto& otherPlayer : state.players)
			{
				if (otherPlayer.team != player.team)
				{
					// Vérifier si cet adversaire a été bumped ou demoed
					if (otherPlayer.eventState.bumped || otherPlayer.eventState.demoed)
					{
						victim = &otherPlayer;
						break;
					}
				}
			}

			// Si pas de victime trouvée, retourner récompense de base (cas rare)
			if (!victim)
			{
				return didDemo ? baseReward * demoMultiplier : baseReward;
			}

			// Déterminer le but adverse (celui qu'on attaque)
			// Blue attaque Orange (Y positif), Orange attaque Blue (Y négatif)
			Vec enemyGoalCenter = (player.team == Team::BLUE)
				? CommonValues::ORANGE_GOAL_CENTER
				: CommonValues::BLUE_GOAL_CENTER;

			// Calculer la distance de la victime à son propre but
			// Plus la victime est proche de son but (qu'on attaque), plus c'est stratégique
			float victimDistToGoal = (victim->pos.To2D() - enemyGoalCenter.To2D()).Length();

			// Normaliser la distance (0 = au but, 1 = très loin)
			float distanceRatio = RS_CLAMP(victimDistToGoal / maxDistanceFromGoal, 0.0f, 1.0f);

			// Inverser pour que proche du but = récompense plus élevée
			float proximityFactor = 1.0f - distanceRatio;

			// Vérifier si la victime était dans la zone du but (très stratégique)
			float goalY = (player.team == Team::BLUE) ? CommonValues::BACK_WALL_Y : -CommonValues::BACK_WALL_Y;
			bool victimInGoalZone = std::abs(victim->pos.y - goalY) < goalZoneThreshold;

			// Bonus si dans la zone du but
			float zoneBonus = victimInGoalZone ? 1.5f : 1.0f;

			// Vérifier si la balle est en position offensive (dans le tiers offensif)
			float offensiveThreshold = CommonValues::BACK_WALL_Y / 3.0f;
			bool ballInOffensiveZone = false;
			if (player.team == Team::BLUE)
			{
				ballInOffensiveZone = state.ball.pos.y > offensiveThreshold;
			}
			else
			{
				ballInOffensiveZone = state.ball.pos.y < -offensiveThreshold;
			}

			// Calculer le bonus balle
			float ballBonus = ballInOffensiveZone ? (1.0f + ballInZoneBonus) : 1.0f;

			// Multiplicateur demo vs bump
			float actionMultiplier = didDemo ? demoMultiplier : 1.0f;

			// Bonus supplémentaire si la victime avait le potentiel de défendre
			// (était entre la balle et le but, face au jeu)
			float defenderBonus = 1.0f;
			Vec ballToGoal = (enemyGoalCenter - state.ball.pos).Normalized();
			Vec victimToBall = (state.ball.pos - victim->pos).Normalized();
			float alignmentWithDefense = ballToGoal.Dot(victimToBall);
			if (alignmentWithDefense > 0.3f && victimInGoalZone)
			{
				// La victime était probablement en train de défendre
				defenderBonus = 1.3f;
			}

			// Calcul final de la récompense
			reward = baseReward *
				(1.0f + proximityFactor) *  // Plus proche du but = plus de reward
				zoneBonus *                  // Bonus zone de but
				ballBonus *                  // Bonus balle offensive
				actionMultiplier *           // Demo vs bump
				defenderBonus;               // Défenseur neutralisé

			// Appliquer le plafond
			reward = RS_MIN(reward, maxReward);

			return reward;
		}
	};
	








		class KickoffProximityReward2v2 : public Reward {
		public:
			float goerReward = 1.0f;      // Reward for player going for kickoff
			float cheaterReward = 0.5f;   // Reward for player staying back

			virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
				// Only during kickoff (ball stationary and centered)
				if (state.ball.vel.Length() > 1.f) return 0.f;

				float playerDistToBall = (player.pos - state.ball.pos).Length();

				// Find teammate and opponents
				const Player* teammate = nullptr;
				float closestOpponentDist = FLT_MAX;

				for (const auto& p : state.players) {
					if (p.team == player.team && p.carId != player.carId) {
						teammate = &p;
					}
					else if (p.team != player.team) {
						float opponentDist = (p.pos - state.ball.pos).Length();
						closestOpponentDist = RS_MIN(closestOpponentDist, opponentDist);
					}
				}

				if (!teammate) return 0.f;

				float teammateDistToBall = (teammate->pos - state.ball.pos).Length();

				// Determine role: are you the "goer" (closest to ball on your team)?
				// Role determination with tiebreaker
				bool isGoer = (playerDistToBall < teammateDistToBall) ||
					(playerDistToBall == teammateDistToBall && player.pos.x < teammate->pos.x);

				if (isGoer) {
					// GOER ROLE: Reward for being faster than opponents
					return (playerDistToBall < closestOpponentDist) ? goerReward : -goerReward;
				}
				else {
					// CHEATER ROLE: IMPROVED - Strategic follow-up positioning, not goal camping
					return CalculateCheaterReward(player, state);
				}
			}

		private:
			float CalculateCheaterReward(const Player& player, const GameState& state) {
				Vec ownGoal = (player.team == Team::BLUE) ?
					CommonValues::BLUE_GOAL_BACK : CommonValues::ORANGE_GOAL_BACK;

				// Define strategic zones for follow-up play
				Vec fieldCenter = Vec(0, 0, 100);
				Vec idealCheaterPos = (ownGoal + fieldCenter) * 0.6f; // 60% towards center from goal

				float distToIdealPos = (player.pos - idealCheaterPos).Length();
				float distToOwnGoal = (player.pos - ownGoal).Length();

				// COMPONENT 1: Ideal positioning reward (strategic follow-up position)
				float idealPosReward = 0.0f;
				float idealRadius = 800.f;
				if (distToIdealPos <= idealRadius) {
					idealPosReward = 0.4f * (1.f - (distToIdealPos / idealRadius));
				}
				else {
					float maxAcceptableDist = 1500.f;
					idealPosReward = 0.4f * RS_MAX(0.f, 1.f - ((distToIdealPos - idealRadius) / maxAcceptableDist));
				}

				// COMPONENT 2: Boost proximity bonus (preparation for next play)
				float boostBonus = CalculateBoostProximityBonus(player, state);

				// COMPONENT 3: Anti-goal-camping penalty (discourages passive play)
				float goalCampPenalty = 0.0f;
				float minDistFromGoal = 1000.f;
				if (distToOwnGoal < minDistFromGoal) {
					goalCampPenalty = -0.2f * (1.f - (distToOwnGoal / minDistFromGoal));
				}

				// COMPONENT 4: Field awareness bonus
				float awarenessBonus = CalculateFieldAwarenessBonus(player, state, ownGoal);

				return RS_CLAMP(idealPosReward + boostBonus + goalCampPenalty + awarenessBonus, -0.5f, 0.5f);
			}

			float CalculateBoostProximityBonus(const Player& player, const GameState& state) {
				// Find closest large boost pad using your framework's boost locations
				// Large boost pads are the ones with z=73.0 (6 total: 4 corners + 2 mid-field)
				float closestBoostDist = FLT_MAX;

				for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
					const Vec& boostPos = CommonValues::BOOST_LOCATIONS[i];

					// Filter for large boost pads (z=73.0 indicates 100% boost pads)
					if (boostPos.z > 72.0f) {
						float dist = (player.pos - boostPos).Length();
						closestBoostDist = RS_MIN(closestBoostDist, dist);
					}
				}

				if (closestBoostDist <= 1000.f) {
					return 0.1f * (1.f - (closestBoostDist / 1000.f));
				}
				return 0.0f;
			}

			float CalculateFieldAwarenessBonus(const Player& player, const GameState& state, const Vec& ownGoal) {
				Vec toBall = (state.ball.pos - player.pos).Normalized();
				Vec toGoal = (ownGoal - player.pos).Normalized();

				float angleDot = toBall.Dot(toGoal);

				// Best when positioned to quickly transition between ball focus and goal defense
				if (angleDot >= -0.7f && angleDot <= 0.2f) {
					return 0.05f;
				}
				return 0.0f;
			}
	};


	
	class ControlReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.ballTouchedStep || !player.prev) {
				return 0;
			}

			// Calculate distance to ball
			float dist = player.pos.Dist(state.ball.pos);

			// Reward is higher the closer the player is to the ball, max reward is 1.
			// The max distance is set based on a rough estimate of dribble distance.
			constexpr float MAX_DIST = 500.f;
			return exp(-0.5 * (dist / MAX_DIST) * (dist / MAX_DIST));
		}
	};

	class ProgressiveDribblePopBumpReward : public Reward {
	private:
		struct PlayerState {
			bool isDribbling = false;
			int dribbleTouches = 0;
			int ticksSinceLastDribbleTouch = 0;
			bool justPopped = false;
			int ticksSincePop = 0;
			bool eligibleForBumpReward = false;
			int ticksSinceEligible = 0;
		};

		std::map<uint32_t, PlayerState> states;

	public:
		float dribbleProgressReward;    // Continuous reward for dribbling
		float popReward;                // Reward for successful pop
		float bumpReward;               // Reward for bump after pop
		float demoReward;               // Reward for demo after pop
		float sequenceCompleteBonus;    // Extra bonus for full sequence

		ProgressiveDribblePopBumpReward(
			float dribbleProgressReward = 0.1f,
			float popReward = 3.0f,
			float bumpReward = 15.0f,
			float demoReward = 25.0f,
			float sequenceCompleteBonus = 5.0f
		) : dribbleProgressReward(dribbleProgressReward),
			popReward(popReward),
			bumpReward(bumpReward),
			demoReward(demoReward),
			sequenceCompleteBonus(sequenceCompleteBonus) {
		}

		virtual void Reset(const GameState& initialState) override {
			states.clear();
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev) return 0.0f;

			auto& pState = states[player.carId];

			pState.ticksSinceLastDribbleTouch++;
			pState.ticksSincePop++;
			pState.ticksSinceEligible++;

			float reward = 0.0f;

			// STEP 1: Reward dribbling progress
			float distToBall = (player.pos - state.ball.pos).Length();
			bool isDribblingNow = player.isOnGround &&
				state.ball.pos.z <= 250.0f &&
				distToBall <= 180.0f &&
				player.vel.Length() >= 300.0f;

			if (isDribblingNow) {
				reward += dribbleProgressReward;

				if (player.ballTouchedStep) {
					if (pState.ticksSinceLastDribbleTouch < 60) {
						pState.dribbleTouches++;
					}
					else {
						pState.dribbleTouches = 1;
					}
					pState.ticksSinceLastDribbleTouch = 0;
				}
			}

			pState.isDribbling = isDribblingNow;

			// STEP 2: Reward pop
			bool wasDribblingRecently = pState.ticksSinceLastDribbleTouch <= 20;
			bool hadEnoughTouches = pState.dribbleTouches >= 2;
			bool ballWentHigh = state.ball.pos.z >= 300.0f &&
				state.prev->ball.pos.z < 300.0f;
			bool ballMovingUp = state.ball.vel.z >= 400.0f;

			if (wasDribblingRecently && hadEnoughTouches && ballWentHigh && ballMovingUp && player.ballTouchedStep) {
				reward += popReward;
				pState.justPopped = true;
				pState.ticksSincePop = 0;
				pState.eligibleForBumpReward = true;
				pState.ticksSinceEligible = 0;
			}

			if (pState.ticksSincePop > 5) {
				pState.justPopped = false;
			}

			if (pState.ticksSinceEligible > 90) {
				pState.eligibleForBumpReward = false;
				pState.dribbleTouches = 0;
			}

			// STEP 3: Reward bump/demo after pop
			if (pState.eligibleForBumpReward &&
				(player.eventState.bump || player.eventState.demo)) {

				bool didDemo = player.eventState.demo;
				reward += didDemo ? demoReward : bumpReward;

				// Full sequence bonus!
				reward += sequenceCompleteBonus;

				pState.eligibleForBumpReward = false;
				pState.dribbleTouches = 0;
			}

			return reward;
		}
	};

	class FlipResetReward : public Reward {
	private:
		struct PlayerState {
			bool lastHasFlip;
			bool resetDetected;
			bool setupActive;
			float lastBallDistance;
			float lastRollAngle;
			float lastAirTime;
			int resetFrame;

			PlayerState()
				: lastHasFlip(false)
				, resetDetected(false)
				, setupActive(false)
				, lastBallDistance(std::numeric_limits<float>::infinity())
				, lastRollAngle(0.0f)
				, lastAirTime(0.0f)
				, resetFrame(0)
			{
			}
		};

		float resetReward;
		float setupBonus;
		float proximityBonus;
		float invertedBonus;
		float weight;
		float minAirHeight;
		float maxBallDistance;
		float invertedThreshold;
		bool debug;

		std::unordered_map<uint32_t, PlayerState> playerStates;

		bool IsInAir(const Player& player) const {
			return player.pos.z > minAirHeight;
		}

		bool IsCarInverted(const Player& player) const {
			Angle angle = Angle::FromRotMat(player.rotMat);
			float rollAngleDeg = std::abs(angle.roll * 180.0f / M_PI);
			return std::abs(rollAngleDeg - 180.0f) < invertedThreshold;
		}

		bool IsCloseToBall(const Player& player, const GameState& state) const {
			float distance = (player.pos - state.ball.pos).Length();
			return distance <= maxBallDistance;
		}

		bool DetectWheelContact(const Player& player, const GameState& state) const {
			if (!IsCloseToBall(player, state)) {
				return false;
			}

			float distance = (player.pos - state.ball.pos).Length();
			return distance <= 100.0f && IsCarInverted(player);
		}

		bool DetectFlipReset(const Player& player, PlayerState& playerState) {
			bool currentHasFlip = player.HasFlipOrJump();
			bool lastHasFlip = playerState.lastHasFlip;

			bool flipResetDetected = !lastHasFlip && currentHasFlip;

			playerState.lastHasFlip = currentHasFlip;

			return flipResetDetected;
		}

		float CalculateSetupQuality(const Player& player, const GameState& state) const {
			float quality = 0.0f;

			if (IsInAir(player)) {
				quality += 0.3f;
			}

			if (IsCarInverted(player)) {
				quality += 0.3f;
			}

			float distance = (player.pos - state.ball.pos).Length();
			if (distance <= maxBallDistance) {
				float proximityScore = 1.0f - (distance / maxBallDistance);
				quality += 0.4f * proximityScore;
			}

			return quality;
		}

	public:
		FlipResetReward(
			float resetReward = 15.0f,
			float setupBonus = 2.0f,
			float proximityBonus = 1.0f,
			float invertedBonus = 1.0f,
			float weight = 1.0f,
			float minAirHeight = 50.0f,
			float maxBallDistance = 150.0f,
			float invertedThreshold = 150.0f,
			bool debug = false)
			: resetReward(resetReward)
			, setupBonus(setupBonus)
			, proximityBonus(proximityBonus)
			, invertedBonus(invertedBonus)
			, weight(weight)
			, minAirHeight(minAirHeight)
			, maxBallDistance(maxBallDistance)
			, invertedThreshold(invertedThreshold)
			, debug(debug)
		{
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				playerStates[player.carId] = PlayerState();
				playerStates[player.carId].lastHasFlip = player.HasFlipOrJump();
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (playerStates.find(player.carId) == playerStates.end()) {
				playerStates[player.carId] = PlayerState();
				playerStates[player.carId].lastHasFlip = player.HasFlipOrJump();
			}

			PlayerState& playerState = playerStates[player.carId];
			float reward = 0.0f;

			bool inAir = IsInAir(player);
			bool inverted = IsCarInverted(player);
			bool closeToBall = IsCloseToBall(player, state);
			bool wheelContact = DetectWheelContact(player, state);

			bool flipResetDetected = DetectFlipReset(player, playerState);

			if (inAir && inverted && closeToBall && wheelContact && flipResetDetected) {
				if (!playerState.resetDetected) {
					reward += resetReward;
					playerState.resetDetected = true;

					if (debug) {
						std::cout << "FLIP RESET DETECTED! Player " << player.carId
							<< " - Reward: " << resetReward << std::endl;
					}
				}
			}
			else if (inAir && inverted && closeToBall) {
				float setupQuality = CalculateSetupQuality(player, state);
				reward += setupBonus * setupQuality;

				if (!playerState.setupActive) {
					playerState.setupActive = true;
				}
			}
			else {
				if (playerState.setupActive) {
					playerState.setupActive = false;
				}
				if (playerState.resetDetected) {
					playerState.resetDetected = false;
				}
			}

			playerState.lastBallDistance = (player.pos - state.ball.pos).Length();

			Angle angle = Angle::FromRotMat(player.rotMat);
			playerState.lastRollAngle = std::abs(angle.roll * 180.0f / M_PI);

			return reward * weight;
		}
	};

	class KickoffFirstTouchReward : public Reward {
	private:
		bool _kickoff_active;
		int _first_touch_player_id_this_tick;

	public:
		KickoffFirstTouchReward() : _kickoff_active(false), _first_touch_player_id_this_tick(-1) {}
		void Reset(const GameState& initial_state) override {
			_kickoff_active = initial_state.ball.vel.LengthSq() < 1.f;
			_first_touch_player_id_this_tick = -1;
		}

		void PreStep(const GameState& state) override {
			_first_touch_player_id_this_tick = -1;

			if (!_kickoff_active) return;
			bool kickoff_ended = false;

			for (const auto& p : state.players) {
				if (p.ballTouchedStep) {
					_first_touch_player_id_this_tick = p.carId;
					kickoff_ended = true;
					break;
				}
			}

			if (!kickoff_ended && state.ball.vel.LengthSq() > 1.f) {
				kickoff_ended = true;
			}

			if (kickoff_ended) {
				_kickoff_active = false;
			}
		}

		float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (_first_touch_player_id_this_tick != -1) {
				if (player.carId == _first_touch_player_id_this_tick) {
					return 1.0f;
				}
				else {
					return -1.0f;
				}
			}

			return 0.0f;
		}
	};

	/**
	 * @brief Rewards the player for executing a 45-degree flick.
	 * NOTE: This is a simplified detection and rewards any flick that sends the ball upward.
	 * A more precise implementation would need to check car-to-ball orientation before the flick.
	 */
	class FlickReward45Degree : public Reward {
	private:
		struct PlayerState {
			bool wasOnGround;
			bool ballWasOnRoof;
			float timeSinceBallOnRoof;
			Vec lastCarPos;

			PlayerState() : wasOnGround(true), ballWasOnRoof(false),
				timeSinceBallOnRoof(0.0f), lastCarPos(0, 0, 0) {
			}
		};

		static constexpr float TICK_SKIP = 4.0f;
		static constexpr float DT = (1.0f / 120.0f) * TICK_SKIP;
		std::unordered_map<uint32_t, PlayerState> playerStates;

	public:
		float minBallSpeed;
		float maxBallSpeed;
		float angleTolerance;
		float roofDistanceMax;
		float maxTimeSinceRoof;

		FlickReward45Degree(
			float minBallSpeed = 1200.0f,
			float maxBallSpeed = 3500.0f,
			float angleTolerance = 20.0f,
			float roofDistanceMax = 150.0f,
			float maxTimeSinceRoof = 0.3f
		) : minBallSpeed(minBallSpeed), maxBallSpeed(maxBallSpeed),
			angleTolerance(angleTolerance), roofDistanceMax(roofDistanceMax),
			maxTimeSinceRoof(maxTimeSinceRoof) {
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				PlayerState state;
				state.wasOnGround = player.isOnGround;
				state.ballWasOnRoof = false;
				state.timeSinceBallOnRoof = 999.0f;
				state.lastCarPos = player.pos;
				playerStates[player.carId] = state;
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			uint32_t carId = player.carId;

			// Initialize if needed
			if (playerStates.find(carId) == playerStates.end()) {
				PlayerState newState;
				newState.wasOnGround = player.isOnGround;
				newState.ballWasOnRoof = false;
				newState.timeSinceBallOnRoof = 999.0f;
				newState.lastCarPos = player.pos;
				playerStates[carId] = newState;
			}

			PlayerState& ps = playerStates[carId];

			// Check if ball is currently on car roof
			Vec carToBall = state.ball.pos - player.pos;
			float distToBall = carToBall.Length();
			Vec carUp = player.rotMat.up;

			// Ball is "on roof" if it's close, above the car, and aligned with car's up vector
			bool ballOnRoofNow = false;
			if (distToBall < roofDistanceMax) {
				float heightAboveCar = carToBall.Dot(carUp);
				if (heightAboveCar > 50.0f && heightAboveCar < 150.0f) {
					// Check if ball is roughly above car (not to the side)
					Vec carToBallHorizontal = carToBall - (carUp * heightAboveCar);
					if (carToBallHorizontal.Length() < 80.0f) {
						ballOnRoofNow = true;
					}
				}
			}

			// Update timing
			if (ballOnRoofNow) {
				ps.timeSinceBallOnRoof = 0.0f;
				ps.ballWasOnRoof = true;
			}
			else {
				ps.timeSinceBallOnRoof += DT;
			}

			float reward = 0.0f;

			// Detect flick: player touched ball recently after having it on roof
			if (player.ballTouchedStep && ps.ballWasOnRoof && ps.timeSinceBallOnRoof < maxTimeSinceRoof) {

				// Check if player just left ground (jumped for flick)
				bool justJumped = ps.wasOnGround && !player.isOnGround;

				if (!state.prev) {
					ps.wasOnGround = player.isOnGround;
					ps.lastCarPos = player.pos;
					return 0.0f;
				}

				// Check ball velocity for 45-degree angle
				float ballSpeed = state.ball.vel.Length();

				if (ballSpeed >= minBallSpeed) {
					Vec ballVelNorm = state.ball.vel.Normalized();

					// Calculate angle from horizontal
					float horizontalSpeed = sqrtf(ballVelNorm.x * ballVelNorm.x + ballVelNorm.y * ballVelNorm.y);
					float angleDeg = atan2f(ballVelNorm.z, horizontalSpeed) * 57.2957795f;

					// Check if angle is near 45 degrees
					float angleDiff = fabsf(angleDeg - 45.0f);

					if (angleDiff <= angleTolerance) {
						// Power bonus (speed-based)
						float powerBonus = RS_MIN(1.0f, ballSpeed / maxBallSpeed);

						// Angle precision bonus
						float anglePrecision = 1.0f - (angleDiff / angleTolerance);

						// Height bonus (higher flicks are better)
						float heightBonus = RS_MIN(1.0f, state.ball.pos.z / 500.0f);

						// Jump bonus (if player jumped for the flick)
						float jumpBonus = justJumped ? 1.5f : 1.0f;

						// Timing bonus (quicker flick after roof = better)
						float timingBonus = 1.0f - (ps.timeSinceBallOnRoof / maxTimeSinceRoof);
						timingBonus = RS_MAX(0.3f, timingBonus);

						reward = powerBonus * anglePrecision * heightBonus * jumpBonus * timingBonus;

						// Reset state after successful flick
						ps.ballWasOnRoof = false;
						ps.timeSinceBallOnRoof = 999.0f;
					}
				}
			}

			// Update state for next frame (ALWAYS, not just when reward is 0!)
			ps.wasOnGround = player.isOnGround;
			ps.lastCarPos = player.pos;

			return reward;
		}
	};

	/**
	 * @brief Rewards the player for using air roll while airborne.
	 */
	class AirRollReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.isOnGround && player.prevAction.roll != 0) {
				return 1.0f;
			}
			return 0;
		}
	};

	/**
	 * @brief Rewards the player for moving towards the closest available boost pad.
	 */
	class BoostSeekingReward : public Reward {
	private:
		struct PlayerState {
			float lastBoost = 100.0f;
			Vec lastPosition = Vec(0, 0, 0);
			bool wasMovingTowardsBoost = false;
			float boostSeekingTime = 0.0f;
			Vec targetBoostPos = Vec(0, 0, 0);
		};
		std::map<uint32_t, PlayerState> playerStates;

		std::vector<Vec> smallBoostPositions;
		std::vector<Vec> bigBoostPositions;

	public:
		float boostPickupReward;
		float boostSeekingRewardVal;
		float boostProximityReward;
		float boostDirectionReward;
		float lowBoostThreshold;
		float maxSeekingDistance;

		BoostSeekingReward(
			float boostPickupReward = 5.0f,
			float boostSeekingRewardVal = 0.8f,
			float boostProximityReward = 1.2f,
			float boostDirectionReward = 1.5f,
			float lowBoostThreshold = 0.3f,
			float maxSeekingDistance = 800.0f
		) : boostPickupReward(boostPickupReward), boostSeekingRewardVal(boostSeekingRewardVal),
			boostProximityReward(boostProximityReward), boostDirectionReward(boostDirectionReward),
			lowBoostThreshold(lowBoostThreshold), maxSeekingDistance(maxSeekingDistance) {
			initializeBoostPositions();
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				PlayerState& state = playerStates[player.carId];
				state.lastBoost = player.boost;
				state.lastPosition = player.pos;
				state.wasMovingTowardsBoost = false;
				state.boostSeekingTime = 0.0f;
				state.targetBoostPos = Vec(0, 0, 0);
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			uint32_t carId = player.carId;

			if (playerStates.find(carId) == playerStates.end()) {
				PlayerState& newState = playerStates[carId];
				newState.lastBoost = player.boost;
				newState.lastPosition = player.pos;
				newState.wasMovingTowardsBoost = false;
				newState.boostSeekingTime = 0.0f;
				newState.targetBoostPos = Vec(0, 0, 0);
			}

			PlayerState& st = playerStates[carId];
			float reward = 0.0f;
			Vec playerPos = player.pos;
			float currentBoost = player.boost;

			if (currentBoost > st.lastBoost) {
				float boostGained = currentBoost - st.lastBoost;
				reward += boostPickupReward * (boostGained / 100.0f);
			}

			if (currentBoost < lowBoostThreshold) {
				Vec closestBoost = findClosestBoost(playerPos);
				float distanceToBoost = (closestBoost - playerPos).Length();

				if (distanceToBoost <= maxSeekingDistance) {
					float proximityFactor = 1.0f - (distanceToBoost / maxSeekingDistance);
					reward += boostProximityReward * proximityFactor;

					if (isMovingTowardsBoost(player, closestBoost)) {
						reward += boostDirectionReward;
						st.wasMovingTowardsBoost = true;
						st.boostSeekingTime += 1.0f / 120.0f;

						if (st.boostSeekingTime > 1.0f) {
							reward += boostSeekingRewardVal * 0.5f;
						}
					}
					else {
						st.wasMovingTowardsBoost = false;
						st.boostSeekingTime = 0.0f;
					}

					st.targetBoostPos = closestBoost;
				}
			}
			else {
				st.boostSeekingTime = 0.0f;
				st.wasMovingTowardsBoost = false;
			}

			st.lastBoost = currentBoost;
			st.lastPosition = playerPos;

			return reward;
		}

	private:
		void initializeBoostPositions() {
			bigBoostPositions = {
				Vec(-3072, -4096, 73),
				Vec(3072, -4096, 73),
				Vec(-3072, 4096, 73),
				Vec(3072, 4096, 73),
				Vec(0, -4240, 73),
				Vec(0, 4240, 73)
			};

			smallBoostPositions.clear();
			for (int x = -2560; x <= 2560; x += 1024) {
				for (int y = -3840; y <= 3840; y += 1280) {
					bool isBigBoost = false;
					for (const auto& bigBoost : bigBoostPositions) {
						if ((Vec((float)x, (float)y, 73) - bigBoost).Length() < 200) {
							isBigBoost = true;
							break;
						}
					}
					if (!isBigBoost) {
						smallBoostPositions.push_back(Vec((float)x, (float)y, 73));
					}
				}
			}
		}

		Vec findClosestBoost(const Vec& playerPos) const {
			Vec closestBoost = Vec(0, 0, 0);
			float minDistance = std::numeric_limits<float>::infinity();

			for (const auto& boostPos : bigBoostPositions) {
				float distance = (boostPos - playerPos).Length();
				if (distance < minDistance) {
					minDistance = distance;
					closestBoost = boostPos;
				}
			}

			for (const auto& boostPos : smallBoostPositions) {
				float distance = (boostPos - playerPos).Length();
				if (distance < minDistance) {
					minDistance = distance;
					closestBoost = boostPos;
				}
			}

			return closestBoost;
		}

		bool isMovingTowardsBoost(const Player& player, const Vec& boostPos) const {
			Vec playerPos = player.pos;
			Vec directionToBoost = (boostPos - playerPos).Normalized();
			Vec playerVelocity = player.vel;

			if (playerVelocity.Length() < 100) {
				return false;
			}

			Vec playerVelocityNorm = playerVelocity.Normalized();
			float alignment = playerVelocityNorm.Dot(directionToBoost);

			return alignment > 0.3f;
		}
	};


	/**
	 * @brief Rewards the player for performing a wavedash.
	 * This is a more robust version that checks for the speed increase from the wavedash.
	 */
	class WaveDashReward2 : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev) return 0;

			// Check for transition from flipping in air to being on ground
			if (player.isOnGround && player.prev->isFlipping && !player.prev->isOnGround) {
				// Reward the speed increase from the wavedash
				float speed_increase = player.vel.Length() - player.prev->vel.Length();
				if (speed_increase > 100) { // Must provide a meaningful speed boost
					return 1.0f;
				}
			}
			return 0;
		}
	};

	/**
	 * @brief Rewards the player for executing a half-flip.
	 * NOTE: A precise half-flip is hard to detect. This rewards quickly turning 180 degrees
	 * while on the ground, which is the primary outcome of a half-flip.
	 */
	class HalfFlipReward2 : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.isOnGround || !player.prev || !player.prev->prev) return 0;

			// Check if the player was recently flipping backwards.
			if (player.prev->hasFlipped && player.prev->prevAction.pitch > 0.5) {
				// Check if the car is now facing the opposite direction.
				float dot = player.rotMat.forward.Dot(player.prev->prev->rotMat.forward);
				if (dot < -0.8) { // Facing nearly the opposite direction
					return 1.0f;
				}
			}
			return 0;
		}
	};

	/**
	 * @brief Rewards a speed-flip.
	 * NOTE: Precise detection is complex. This implementation rewards reaching supersonic
	 * very quickly from a near-standstill, a key outcome of a speed-flip.
	 */
	class AdvancedSpeedFlipReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev || player.isSupersonic == player.prev->isSupersonic) return 0;

			// If we just became supersonic and our previous speed was low
			if (player.isSupersonic && player.prev->vel.Length() < 500) {
				return 1.0f;
			}

			return 0;
		}
	};

	/**
	 * @brief Rewards doing a speed-flip on kickoff.
	 * NOTE: This rewards getting to the ball quickly on kickoff as a proxy for a good kickoff.
	 */
	class KickoffSpeedFlipRewardV2 : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Check if it's a kickoff scenario (ball is at center)
			if (state.ball.pos.Length2D() < 10) {
				if (player.ballTouchedStep) {
					// Reward is inversely proportional to the time it took to touch the ball
					return 2.5f - state.ball.pos.z / (CommonValues::BALL_RADIUS * 2);
				}
			}
			return 0;
		}
	};

	/**
	 * @brief Rewards the player for chain-dashing on the wall.
	 */
	class InfiniteWalldashRewardV2 : public Reward {
	private:
		struct PlayerState {
			bool wasOnWall = false;
			bool dashUsed = false;
			bool resetObtained = false;
			int consecutiveWallDashes = 0;
			float lastDashTime = 0.0f;
			Vec lastWallPosition = Vec(0, 0, 0);
			bool hadFlipBefore = false;
			float wallStayTime = 0.0f;
			int dashCount = 0;
			float totalDashTime = 0.0f;
			bool isSpamming = false;
		};
		std::map<uint32_t, PlayerState> playerStates;

	public:
		float wallHeightThreshold;
		float maxTimeBetweenDashAndReset;
		float dashReward;
		float resetReward;
		float consecutiveBonus;
		float wallStayPenalty;
		float spamBonus;
		float speedBonus;
		float frequencyBonus;
		bool debug;

		InfiniteWalldashRewardV2(
			float wallHeightThreshold = 100.0f,
			float maxTimeBetweenDashAndReset = 4.0f,
			float dashReward = 12.0f,
			float resetReward = 16.0f,
			float consecutiveBonus = 8.0f,
			//	float wallStayPenalty = -0.2f,
			float spamBonus = 20.0f,
			float speedBonus = 120.0f,
			float frequencyBonus = 6.0f,
			bool debug = false
		) : wallHeightThreshold(wallHeightThreshold),
			maxTimeBetweenDashAndReset(maxTimeBetweenDashAndReset),
			dashReward(dashReward), resetReward(resetReward),
			consecutiveBonus(consecutiveBonus), wallStayPenalty(wallStayPenalty),
			spamBonus(spamBonus), speedBonus(speedBonus),
			frequencyBonus(frequencyBonus), debug(debug) {
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				PlayerState& state = playerStates[player.carId];
				state.wasOnWall = false;
				state.dashUsed = false;
				state.resetObtained = false;
				state.consecutiveWallDashes = 0;
				state.lastDashTime = 0.0f;
				state.lastWallPosition = Vec(0, 0, 0);
				state.hadFlipBefore = player.HasFlipOrJump();
				state.wallStayTime = 0.0f;
				state.dashCount = 0;
				state.totalDashTime = 0.0f;
				state.isSpamming = false;
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev) return 0.0f;

			uint32_t carId = player.carId;
			if (playerStates.find(carId) == playerStates.end()) {
				PlayerState& newState = playerStates[carId];
				newState.wasOnWall = false;
				newState.dashUsed = false;
				newState.resetObtained = false;
				newState.consecutiveWallDashes = 0;
				newState.lastDashTime = 0.0f;
				newState.lastWallPosition = Vec(0, 0, 0);
				newState.hadFlipBefore = player.HasFlipOrJump();
				newState.wallStayTime = 0.0f;
				newState.dashCount = 0;
				newState.totalDashTime = 0.0f;
				newState.isSpamming = false;
			}

			PlayerState& st = playerStates[carId];
			float reward = 0.0f;
			float currentTime = 0.0f; // Simuler le temps

			// Vérifier si le joueur est sur un mur
			bool isOnWall = IsOnWall(player.pos);
			bool hasFlip = player.HasFlipOrJump();

			// === PHASE 1: DÉTECTION DU DASH DEPUIS LE MUR ===
			if (DetectDashFromWall(player, st)) {
				reward += dashReward;
				st.dashUsed = true;
				st.lastDashTime = currentTime;
				st.dashCount++;

				if (debug) {
					printf("[WallDashV2] car_id=%d DASH #%d! Reward: %.2f\n",
						carId, st.dashCount, dashReward);
				}
			}

			// === PHASE 2: DÉTECTION DU RESET SUR LE MUR ===
			if (DetectWallReset(player, st)) {
				reward += resetReward;
				st.resetObtained = true;
				st.consecutiveWallDashes++;

				// Bonus pour les wall dashes consécutifs
				if (st.consecutiveWallDashes > 1) {
					float consecutiveReward = consecutiveBonus * st.consecutiveWallDashes;
					reward += consecutiveReward;

					if (debug) {
						printf("[WallDashV2] car_id=%d WALL DASH CONSÉCUTIF #%d! Bonus: %.2f\n",
							carId, st.consecutiveWallDashes, consecutiveReward);
					}
				}

				// === BONUS SPAM WALLDASH ===
				// Plus le bot fait de dashes rapidement, plus il est récompensé
				float timeSinceDash = currentTime - st.lastDashTime;
				if (timeSinceDash < 1.0f) { // Dash rapide
					float spamReward = spamBonus * (1.0f - timeSinceDash);
					reward += spamReward;

					if (debug) {
						printf("[WallDashV2] car_id=%d SPAM WALLDASH! Temps: %.2fs, Bonus: %.2f\n",
							carId, timeSinceDash, spamReward);
					}
				}

				// === BONUS FRÉQUENCE ===
				// Récompense pour faire beaucoup de dashes
				if (st.dashCount >= 3) {
					float frequencyReward = frequencyBonus * (st.dashCount / 3.0f);
					reward += frequencyReward;

					if (debug && st.dashCount % 3 == 0) {
						printf("[WallDashV2] car_id=%d FRÉQUENCE ÉLEVÉE! %d dashes, Bonus: %.2f\n",
							carId, st.dashCount, frequencyReward);
					}
				}

				// === BONUS VITESSE ===
				// Récompense pour la vitesse du mouvement
				float playerSpeed = player.vel.Length();
				if (playerSpeed > 800.0f) {
					float speedReward = speedBonus * (playerSpeed / 1000.0f);
					reward += speedReward;

					if (debug && playerSpeed > 1200.0f) {
						printf("[WallDashV2] car_id=%d VITESSE ÉLEVÉE! %.0f, Bonus: %.2f\n",
							carId, playerSpeed, speedReward);
					}
				}

				if (debug) {
					printf("[WallDashV2] car_id=%d RESET SUR LE MUR! Reward: %.2f\n", carId, resetReward);
				}
			}

			// === PHASE 3: GESTION DU TEMPS SUR LE MUR ===
			if (isOnWall) {
				st.wallStayTime += 1.0f / 120.0f; // 120 ticks par seconde

				// Pénalité plus douce pour le spam walldash
				if (st.wallStayTime > 2.0f) { // Après 2 secondes (plus permissif)
					reward += wallStayPenalty;

					if (debug && st.wallStayTime > 3.0f) {
						printf("[WallDashV2] car_id=%d RESTE TROP LONGTEMPS SUR LE MUR! Pénalité: %.2f\n",
							carId, wallStayPenalty);
					}
				}
			}
			else {
				st.wallStayTime = 0.0f;
			}

			// === PHASE 4: RÉINITIALISATION SI LE JOUEUR TOUCHE LE SOL ===
			if (player.isOnGround) {
				// Récompense finale basée sur la performance
				if (st.dashCount >= 5) {
					float finalBonus = 10.0f;
					reward += finalBonus;

					if (debug) {
						printf("[WallDashV2] car_id=%d 🎯 SESSION WALLDASH TERMINÉE! %d dashes, Bonus final: %.2f\n",
							carId, st.dashCount, finalBonus);
					}
				}

				// Reset des compteurs
				st.consecutiveWallDashes = 0;
				st.dashUsed = false;
				st.resetObtained = false;
				st.wallStayTime = 0.0f;
				st.dashCount = 0;
				st.totalDashTime = 0.0f;
				st.isSpamming = false;
			}

			// Mise à jour de l'état
			st.wasOnWall = isOnWall;
			st.hadFlipBefore = hasFlip;
			if (isOnWall) {
				st.lastWallPosition = player.pos;
			}

			return reward;
		}

	private:
		bool IsOnWall(const Vec& playerPos) {
			// Doit être assez haut
			if (playerPos.z < wallHeightThreshold) {
				return false;
			}

			// Vérifier la proximité avec les murs latéraux (Y)
			float sideWallY = fabsf(fabsf(playerPos.y) - CommonValues::BACK_WALL_Y / 2);
			if (sideWallY < 150.0f) {
				return true;
			}

			// Vérifier la proximité avec les murs de fond (X)
			float endWallX = fabsf(fabsf(playerPos.x) - CommonValues::SIDE_WALL_X / 2);
			if (endWallX < 150.0f) {
				return true;
			}

			return false;
		}

		bool DetectDashFromWall(const Player& player, PlayerState& st) {
			// Conditions pour détecter un dash depuis le mur :
			// 1. Était sur un mur
			// 2. Avait un flip
			// 3. N'a plus de flip (l'a utilisé)
			// 4. N'est plus sur le mur (a sauté)

			return (st.wasOnWall &&
				st.hadFlipBefore &&
				!player.HasFlipOrJump() &&
				!IsOnWall(player.pos));
		}

		bool DetectWallReset(const Player& player, PlayerState& st) {
			// Conditions pour détecter un reset sur le mur :
			// 1. Avait utilisé un dash (n'avait plus de flip)
			// 2. Est maintenant sur un mur
			// 3. A récupéré son flip

			return (st.dashUsed &&
				IsOnWall(player.pos) &&
				player.HasFlipOrJump());
		}
	};

	/**
	 * @brief Rewards scoring via a ceiling shot.
	 */
	class CeilingShotReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (player.eventState.goal && player.prev) {
				// Crude check: was the player on the ceiling recently?
				// A proper implementation would need stateful tracking per player.
				const Player* p = player.prev;
				for (int i = 0; i < 120 * 3 && p != nullptr; ++i, p = p->prev) { // Check last 3 seconds
					if (p->worldContact.hasContact && p->worldContact.contactNormal.z < -0.9f) {
						return 1.0f;
					}
				}
			}
			return 0;
		}
	};


	class FlickeReward : public Reward {
	public:
		// Tweakable parameters for flick detection
		float min_dribble_height;
		float max_dribble_height;
		float max_dribble_dist;
		float max_dribble_rel_vel;
		float min_flick_speed_boost;
		float setup_reward;
		float flick_reward_scaler;

	private:
		// We'll track players who have just jumped with the ball and are ready to flick.
		// We store the tick count of when they jumped to implement a timeout.
		std::unordered_map<uint32_t, uint64_t> players_in_flick_setup;
		static constexpr uint64_t FLICK_TIMEOUT_TICKS = 120; // 1 second timeout to complete the flick

		// Helper to determine if a player is in a stable dribble state
		bool IsDribbling(const Player& player, const GameState& state) {
			if (!player.isOnGround) return false;

			// Check ball height relative to the ground
			if (state.ball.pos.z < min_dribble_height || state.ball.pos.z > max_dribble_height) {
				return false;
			}

			// Check horizontal distance from car to ball
			if (player.pos.Dist2D(state.ball.pos) > max_dribble_dist) {
				return false;
			}

			// Check relative velocity
			if ((player.vel - state.ball.vel).Length() > max_dribble_rel_vel) {
				return false;
			}

			return true;
		}

	public:
		FlickeReward(
			float setup_reward = 1.0f,
			float flick_reward_scaler = 4.0f,
			float min_dribble_height = 115.f,    // A bit above ball radius + car height offset
			float max_dribble_height = 200.f,
			float max_dribble_dist = 150.f,      // Ball should be close
			float max_dribble_rel_vel = 500.f,   // Ball and car should move together
			float min_flick_speed_boost = 200.f  // Must add at least this much speed
		) : setup_reward(setup_reward),
			flick_reward_scaler(flick_reward_scaler),
			min_dribble_height(min_dribble_height),
			max_dribble_height(max_dribble_height),
			max_dribble_dist(max_dribble_dist),
			max_dribble_rel_vel(max_dribble_rel_vel),
			min_flick_speed_boost(min_flick_speed_boost)
		{
		}

		virtual void Reset(const GameState& initialState) override {
			players_in_flick_setup.clear();
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// We need previous states to detect state changes
			if (!player.prev || !player.prev->prev || !state.prev || !state.prev->prev) {
				return 0.0f;
			}

			float reward = 0.0f;
			uint32_t carId = player.carId;

			auto it = players_in_flick_setup.find(carId);
			if (it != players_in_flick_setup.end()) {
				// Player was in flick setup state
				uint64_t jump_tick = it->second;

				// Check for timeout or landing
				if (player.isOnGround || state.lastTickCount > jump_tick + FLICK_TIMEOUT_TICKS) {
					players_in_flick_setup.erase(it);
				}
				// Check if they flicked and hit the ball
				else if (player.ballTouchedStep && player.isFlipping && !player.HasFlipOrJump()) {
					// Get ball speed from before the jump to measure speed increase
					float ball_speed_before_jump = state.prev->prev->ball.vel.Length();
					float ball_speed_after_flick = state.ball.vel.Length();

					float delta_speed = ball_speed_after_flick - ball_speed_before_jump;

					if (delta_speed > min_flick_speed_boost) {
						// Reward is scaled by how much speed was added
						reward = (delta_speed / CommonValues::CAR_MAX_SPEED) * flick_reward_scaler;
					}
					players_in_flick_setup.erase(it);
				}
			}
			else {
				// Player was not in setup, check if they are starting a flick now
				bool was_dribbling = IsDribbling(*player.prev, *state.prev);
				bool just_jumped = !player.isOnGround && player.prev->isOnGround;

				if (was_dribbling && just_jumped) {
					// Player has initiated a flick, give setup reward
					players_in_flick_setup[carId] = state.lastTickCount;
					reward = setup_reward;
				}
			}

			return reward;
		}
	};



	/**
	 * @brief A generic flick reward structure.
	 * NOTE: Specific named flicks are too complex to detect without a dedicated state machine.
	 * This rewards any flick where the ball gains significant vertical velocity.
	 */
	class GenericFlickReward : public Reward {
	private:
		float lastBallVelZ = 0.0f;

	public:
		void Reset(const GameState& initialState) override {
			lastBallVelZ = initialState.ball.vel.z;
		}

		float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			float reward = 0.0f;

			// Detect flip/flick-like motion: off-ground and angular velocity spike
			bool flicking = !player.isOnGround && fabs(player.angVel.x) > 2.5f;

			if (player.ballTouchedStep && flicking) {
				float zVelChange = state.ball.vel.z - lastBallVelZ;

				// Reward sudden upward acceleration of the ball
				if (zVelChange > 300.f) {
					reward = RS_CLAMP(zVelChange / (CommonValues::CAR_MAX_SPEED / 2.f), 0.f, 3.f);
				}
			}

			// Cache current ball z velocity for next frame
			lastBallVelZ = state.ball.vel.z;
			return reward;
		}
	};

	class SwiftGroundDribbleReward : public Reward {
	public:
		const float BALL_RADIUS = 92.75f;
		float reward = 0.f;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (state.ball.pos.z > 100
				&& (BALL_RADIUS + 20) < state.ball.pos.z && state.ball.pos.z < (BALL_RADIUS + 200)
				&& std::abs(std::abs(player.pos.x) - std::abs(state.ball.pos.x)) < 150
				&& std::abs(std::abs(player.pos.y) - std::abs(state.ball.pos.y)) < 150) {

				reward += 0.2f;

				if (player.ballTouchedStep) {
					reward += 0.8f;
				}
			}

			return reward;
		}
	};

	// Named flicks just inherit from the generic one for this implementation
	
	class EnhancedDiagonalFlickReward : public GenericFlickReward {};

	/**
	 * @brief Rewards making a defensive touch on the backboard.
	 */
	class BackboardDefenseReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.ballTouchedStep) return 0;

			bool is_on_own_backboard =
				(abs(player.pos.y) > CommonValues::BACK_WALL_Y - 400) &&
				(player.pos.z > CommonValues::GOAL_HEIGHT) &&
				(RS_SGN(player.pos.y) == (player.team == Team::ORANGE ? 1 : -1));

			if (is_on_own_backboard) {
				Vec own_goal_pos = player.team == Team::BLUE ? CommonValues::BLUE_GOAL_BACK : CommonValues::ORANGE_GOAL_BACK;
				Vec dir_from_goal = (state.ball.pos - own_goal_pos).Normalized();

				// Reward clearing the ball away from goal
				return state.ball.vel.Dot(dir_from_goal) / CommonValues::BALL_MAX_SPEED;
			}

			return 0;
		}
	};

	class kuesresetreward : public Reward {
	public:
		float minHeight;
		float maxDist;

		kuesresetreward(float minHeight = 150.f, float maxDist = 300.f) :
			minHeight(minHeight), maxDist(maxDist) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (player.isOnGround || player.HasFlipOrJump()) {
				return 0.f;
			}

			if (player.pos.z < minHeight) {
				return 0.f;
			}

			if (player.rotMat.up.z >= 0) {
				return 0.f;
			}

			float distToBall = (player.pos - state.ball.pos).Length();
			if (distToBall > maxDist) {
				return 0.f;
			}

			Vec dirToBall = (state.ball.pos - player.pos).Normalized();
			Vec relVel = player.vel - state.ball.vel;
			float approachSpeed = relVel.Dot(dirToBall);
			if (approachSpeed <= 0) {
				return 0.f;
			}

			float normSpeed = RS_CLAMP(approachSpeed / CommonValues::CAR_MAX_SPEED, 0.f, 1.f);
			float normAlign = ((-player.rotMat.up).Dot(dirToBall) + 1.f) / 2.f;
			float normDist = 1.f - RS_CLAMP(distToBall / maxDist, 0.f, 1.f);

			return std::min({ normSpeed, normAlign, normDist });
		}
	};


	class BouncyAirDribbleReward : public Reward {
	public:
		float bouncyDistThreshold;
		float bouncyVelThreshold;
		float minHeightForDribble;
		float minSpeedTowardGoal;
		float maxDistFromBall;
		float minPlayerToBallUpwards;
		float nearGoalShutoffTime;

		BouncyAirDribbleReward(
			float bouncyDist = 10.f, float bouncyVel = 25.f, float minHeight = 200.f,
			float minSpeed = 500.f, float maxDist = 300.f, float minPlayerToBallZ = 0.1f,
			float shutoffTime = 1.f) :
			bouncyDistThreshold(bouncyDist), bouncyVelThreshold(bouncyVel), minHeightForDribble(minHeight),
			minSpeedTowardGoal(minSpeed), maxDistFromBall(maxDist), minPlayerToBallUpwards(minPlayerToBallZ),
			nearGoalShutoffTime(shutoffTime) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {

			Vec opponentGoal = (player.team == Team::BLUE) ?
				CommonValues::ORANGE_GOAL_CENTER : CommonValues::BLUE_GOAL_CENTER;

			bool isBouncy = false;
			float distToBall = (player.pos - state.ball.pos).Length();
			if (distToBall > bouncyDistThreshold) {
				isBouncy = true;
			}
			else {
				float relVelMag = (player.vel - state.ball.vel).Length();
				if (relVelMag > bouncyVelThreshold) {
					isBouncy = true;
				}
			}

			if (!isBouncy) {
				return 0.f;
			}

			bool heightConditionMet = false;
			if (state.ball.pos.z > minHeightForDribble) {
				heightConditionMet = true;
			}
			else if (player.vel.z > 0 && state.ball.vel.z > 0) {
				heightConditionMet = true;
			}

			if (!heightConditionMet) return 0.f;

			if (distToBall > maxDistFromBall) return 0.f;

			float distToGoal2D = (player.pos - opponentGoal).Length2D();
			float minBoostRequired = 15.f * std::pow(distToGoal2D / 5000.f, 1.75f);
			if (player.boost < minBoostRequired) return 0.f;

			Vec playerToBallDir = (state.ball.pos - player.pos).Normalized();
			if (playerToBallDir.z < minPlayerToBallUpwards) return 0.f;

			Vec ballToGoalDir = (opponentGoal - state.ball.pos).Normalized();
			if (player.vel.Dot(ballToGoalDir) < minSpeedTowardGoal || state.ball.vel.Dot(ballToGoalDir) < minSpeedTowardGoal) {
				return 0.f;
			}

			Vec playerToBallDir2D = Vec(playerToBallDir.x, playerToBallDir.y, 0).Normalized();
			Vec ballToGoalDir2D = Vec(ballToGoalDir.x, ballToGoalDir.y, 0).Normalized();
			if (playerToBallDir2D.Dot(ballToGoalDir2D) < 0.75) return 0.f; //near the goal value

			float yDistToGoal = std::abs(state.ball.pos.y - opponentGoal.y);
			float timeToGoal = yDistToGoal / std::max(1.f, std::abs(state.ball.vel.y));

			if (timeToGoal > 0) {
				float gravityZ = state.lastArena ? state.lastArena->GetMutatorConfig().gravity.z : -650.f;
				float heightAtGoal = state.ball.pos.z + state.ball.vel.z * timeToGoal + 0.5f * gravityZ * timeToGoal * timeToGoal;
				if (heightAtGoal > CommonValues::GOAL_HEIGHT + 100) return 0.f;
			}

			if (timeToGoal < nearGoalShutoffTime) {
				return 0.f;
			}

			return 1.f;
		}
	};

	/**
	 * @brief A complex reward for performing air dribbles.
	 * It rewards being close to the ball in the air, maintaining a controlled speed,
	 * touching the ball multiple times, and using air roll.
	 */
	class AirDribbleWithRollReward : public Reward {
	public:
		AirDribbleWithRollReward(
			float minHeight = 326.0f,
			float maxDistance = 250.0f,
			float proximityWeight = 3.0f,
			float velocityWeight = 3.0f,
			float heightWeight = 1.0f,
			float touchBonus = 3.0f,
			float boostThreshold = 0.2f,
			float heightTarget = 1000.0f,
			float heightTargetWeight = 1.0f,
			float upwardBonus = 0.4f,
			float airRollWeight = 0.5f,
			float speedWeight = 1.0f,
			float boostEfficiencyWeight = 1.2f,
			float goalDirectionWeight = 10.5f, // Bonus for moving towards opponent goal
			float awayFromGoalPenaltyWeight = 0.f // Penalty for moving away from opponent goal
		) : minHeight(minHeight), maxDistance(maxDistance),
			proximityWeight(proximityWeight), velocityWeight(velocityWeight),
			heightWeight(heightWeight), touchBonus(touchBonus),
			boostThreshold(boostThreshold), heightTarget(heightTarget),
			heightTargetWeight(heightTargetWeight), upwardBonus(upwardBonus),
			airRollWeight(airRollWeight), speedWeight(speedWeight),
			boostEfficiencyWeight(boostEfficiencyWeight),
			goalDirectionWeight(goalDirectionWeight),
			awayFromGoalPenaltyWeight(awayFromGoalPenaltyWeight) {
		}

		inline virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				playerStates[player.carId] = { player.ballTouchedTick, 0, 0.0f, 0.0f };
			}
		}

		inline virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (playerStates.find(player.carId) == playerStates.end()) {
				playerStates[player.carId] = { player.ballTouchedTick, 0, 0.0f, 0.0f };
			}
			auto& pState = playerStates[player.carId];

			auto resetState = [&]() {
				pState.airTouchStreak = 0;
				pState.lastBallTouch = player.ballTouchedStep;
				pState.lastRollInput = 0.0f;
				pState.directionChanges = 0.0f;
				return 0.0f;
				};

			bool isAirborne = !player.isOnGround &&
				player.pos.z >= minHeight &&
				state.ball.pos.z >= minHeight;

			if (!isAirborne) return resetState();

			Vec distVec = state.ball.pos - player.pos;
			float dist = distVec.Length();
			if (dist > maxDistance) return resetState();

			float proximityFactor = std::max(0.0f, 1.0f - (dist - CommonValues::BALL_RADIUS) / std::max(1e-6f, maxDistance - CommonValues::BALL_RADIUS));
			float proxReward = proximityFactor * proximityWeight;

			float velReward = 0.0f;
			if (dist > 1e-6f) {
				Vec dirToBall = distVec / dist;
				float speedTowardsBall = player.vel.Dot(dirToBall);
				if (speedTowardsBall > 0) {
					velReward = (speedTowardsBall / CommonValues::CAR_MAX_SPEED) * velocityWeight;
				}
			}

			float heightFactor = std::max(0.0f, (player.pos.z - minHeight) / std::max(1e-6f, CommonValues::CEILING_Z - minHeight));
			float heightReward = heightFactor * heightWeight;

			float heightDiff = abs(player.pos.z - heightTarget);
			float heightScale = std::max(0.0f, 1.0f - heightDiff / heightTarget);
			float heightTargetReward = heightScale * heightTargetWeight;

			float controlledSpeedReward = 0.0f;
			float playerSpeed = player.vel.Length();
			constexpr float OPTIMAL_MIN_SPEED = 800.0f;
			constexpr float OPTIMAL_MAX_SPEED = 1400.0f;
			if (playerSpeed >= OPTIMAL_MIN_SPEED && playerSpeed <= OPTIMAL_MAX_SPEED) {
				controlledSpeedReward = speedWeight;
			}
			else if (playerSpeed > OPTIMAL_MAX_SPEED) {
				float excessSpeedPenalty = std::min(0.5f, (playerSpeed - OPTIMAL_MAX_SPEED) / 1000.0f);
				controlledSpeedReward = -excessSpeedPenalty * speedWeight;
			}

			float boostConservationReward = 0.0f;
			if (player.boost > 30.f) {
				float boostFactor = powf(player.boost / 100.f, 0.7f);
				boostConservationReward = boostFactor * boostEfficiencyWeight;
			}

			Vec targetPos = (player.team == Team::BLUE) ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;
			Vec dirToGoal = (targetPos - player.pos).Normalized();
			float goalAlignment = dirToGoal.Dot(player.vel.Normalized());

			// **Bonus for dribbling towards the opponent's goal**
			float goalDirectionReward = std::max(0.f, goalAlignment) * goalDirectionWeight;

			// **Penalty for dribbling away from the opponent's goal**
			float awayPenalty = std::min(0.f, goalAlignment) * awayFromGoalPenaltyWeight; // awayFromGoalPenaltyWeight should be positive

			float baseReward = proxReward + velReward + heightReward + heightTargetReward + controlledSpeedReward + boostConservationReward + goalDirectionReward + awayPenalty;
			float totalWeight = proximityWeight + velocityWeight + heightWeight + heightTargetWeight + speedWeight + boostEfficiencyWeight + goalDirectionWeight + awayFromGoalPenaltyWeight;

			float airRollReward = 0.0f;
			float currentRollInput = player.prevAction.roll;

			if (abs(currentRollInput) > 0.1f && abs(pState.lastRollInput) > 0.1f) {
				if (currentRollInput * pState.lastRollInput < 0) {
					pState.directionChanges += 1.0f;
				}
			}

			if (pState.directionChanges > 0) {
				pState.directionChanges = std::max(0.0f, pState.directionChanges - 0.05f);
			}

			pState.lastRollInput = currentRollInput;
			bool isWiggling = pState.directionChanges > 2.0f;

			if (!isWiggling && abs(currentRollInput) > 0.1f) {
				float baseRollReward = airRollWeight;
				if (currentRollInput > 0.1f) baseRollReward *= 1.5f;
				else baseRollReward *= 0.3f;

				if (player.prevAction.boost > 0 && player.boost > (boostThreshold * 100)) {
					baseRollReward *= 1.4f;
				}
				airRollReward = baseRollReward;
			}

			baseReward += airRollReward;
			totalWeight += 1.0f;

			float normalizedBaseReward = baseReward / std::max(1e-6f, totalWeight);
			float finalReward = normalizedBaseReward;

			if (player.vel.z > 0) finalReward += upwardBonus;

			Vec carUp = player.rotMat.up;
			bool belowBall = player.pos.z < state.ball.pos.z - 30;
			bool facingBall = dist > 1e-6f && distVec.Dot(carUp) / dist > 0.7f;
			if (belowBall && facingBall) finalReward += 0.2f;

			if (!pState.lastBallTouch && player.ballTouchedStep) {
				pState.airTouchStreak++;
				float touchMultiplier = 1.0f;

				if (playerSpeed >= OPTIMAL_MIN_SPEED && playerSpeed <= OPTIMAL_MAX_SPEED) {
					touchMultiplier *= 2.5f;
				}
				else if (playerSpeed > OPTIMAL_MAX_SPEED) {
					float speedPenalty = std::min(0.7f, (playerSpeed - OPTIMAL_MAX_SPEED) / 1000.0f);
					touchMultiplier *= std::max(0.1f, 1.5f - speedPenalty);
				}
				else {
					touchMultiplier *= 1.2f;
				}

				if (player.boost > (boostThreshold * 100)) touchMultiplier *= 1.8f;
				if (dist < CommonValues::BALL_RADIUS + 150.0f) touchMultiplier *= 1.4f;

				float touchReward = touchBonus * pState.airTouchStreak * touchMultiplier;
				finalReward += touchReward;
			}

			pState.lastBallTouch = player.ballTouchedStep;

			float heightScaling = state.ball.pos.z / 1000.0f;
			finalReward *= std::max(0.1f, std::min(2.0f, heightScaling));

			return finalReward;
		}

	private:
		struct PlayerState {
			bool lastBallTouch;
			int airTouchStreak;
			float lastRollInput;
			float directionChanges;
		};

		std::map<uint32_t, PlayerState> playerStates;

		// Reward parameters
		float minHeight;
		float maxDistance;
		float proximityWeight;
		float velocityWeight;
		float heightWeight;
		float touchBonus;
		float boostThreshold;
		float heightTarget;
		float heightTargetWeight;
		float upwardBonus;
		float airRollWeight;
		float speedWeight;
		float boostEfficiencyWeight;
		float goalDirectionWeight;
		float awayFromGoalPenaltyWeight;
	};

	class AirTouchReward : public Reward {
	public:
		const float MAX_TIME_IN_AIR = 1.75f;

		AirTouchReward() {}

		virtual void Reset(const GameState& initialState) override {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (player.ballTouchedStep) {
				float airTimeFraction = std::min(player.airTime, MAX_TIME_IN_AIR) / MAX_TIME_IN_AIR;
				float heightFraction = state.ball.pos.z / CommonValues::CEILING_Z;

				return std::min(airTimeFraction, heightFraction);
			}
			return 0.0f;
		}
	};


	class ContinuousFlipResetReward : public Reward {
	public:
		float minHeight;
		float maxDist;

		ContinuousFlipResetReward(float minHeight = 150.f, float maxDist = 300.f) :
			minHeight(minHeight), maxDist(maxDist) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (player.isOnGround || player.HasFlipOrJump()) {
				return 0.f;
			}

			if (player.pos.z < minHeight) {
				return 0.f;
			}

			if (player.rotMat.up.z >= 0) {
				return 0.f;
			}

			float distToBall = (player.pos - state.ball.pos).Length();
			if (distToBall > maxDist) {
				return 0.f;
			}

			Vec dirToBall = (state.ball.pos - player.pos).Normalized();
			Vec relVel = player.vel - state.ball.vel;
			float approachSpeed = relVel.Dot(dirToBall);
			if (approachSpeed <= 0) {
				return 0.f;
			}

			float normSpeed = RS_CLAMP(approachSpeed / CommonValues::CAR_MAX_SPEED, 0.f, 1.f);
			float normAlign = ((-player.rotMat.up).Dot(dirToBall) + 1.f) / 2.f;
			float normDist = 1.f - RS_CLAMP(distToBall / maxDist, 0.f, 1.f);

			// Calculate the reward
			float reward = std::min({ normSpeed, normAlign, normDist });
			return reward;
		}
	};



	class FlipResetEventReward : public Reward {
	public:
		float minHeight;
		float maxDist;
		float minUpZ;

		FlipResetEventReward(float minHeight = 150.f, float maxDist = 150.f, float minUpZ = -0.7f) :
			minHeight(minHeight), maxDist(maxDist), minUpZ(minUpZ) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev) return 0.f;

			bool gotReset = !player.prev->isOnGround && player.HasFlipOrJump() && !player.prev->HasFlipOrJump();

			if (gotReset) {

				if (player.pos.z > minHeight && player.rotMat.up.z < minUpZ && (player.pos - state.ball.pos).Length() < maxDist) {
					float reward = 1.f;
					// Print to console because the reward is positive
					std::cout << "FlipResetEventReward: " << reward << std::endl;
					return reward;
				}
			}

			return 0.f;
		}
	};


	class ResetShotReward : public Reward {
	private:

		std::map<uint32_t, uint64_t> _tickCountWhenResetObtained;

	public:
		ResetShotReward() {}

		virtual void Reset(const GameState& initial_state) override {
			_tickCountWhenResetObtained.clear();
		}

		virtual void PreStep(const GameState& state) override {
			if (!state.lastArena) return;
			for (const auto& player : state.players) {
				if (!player.prev) continue;

				bool gotReset = !player.prev->isOnGround && player.HasFlipOrJump() && !player.prev->HasFlipOrJump();
				if (gotReset) {
					_tickCountWhenResetObtained[player.carId] = state.lastArena->tickCount;
				}
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev || !state.prev) return 0.f;

			auto it = _tickCountWhenResetObtained.find(player.carId);
			if (it == _tickCountWhenResetObtained.end()) {
				return 0.f;
			}

			bool flipWasUsedForTouch = player.ballTouchedStep &&
				!player.isOnGround &&
				!player.hasJumped &&
				player.prev->HasFlipOrJump() &&
				!player.HasFlipOrJump();

			if (flipWasUsedForTouch) {

				float hitForce = (state.ball.vel - state.prev->ball.vel).Length();
				float ballSpeed = state.ball.vel.Length();
				float baseReward = (hitForce + ballSpeed) / (CommonValues::CAR_MAX_SPEED + CommonValues::BALL_MAX_SPEED);

				uint64_t ticksSinceReset = state.lastArena->tickCount - it->second;
				float timeSinceReset = ticksSinceReset * CommonValues::TICK_TIME;
				float timeBonus = 1.f + std::log1p(timeSinceReset);

				_tickCountWhenResetObtained.erase(it);

				// Calculate the reward
				float reward = baseReward * timeBonus;

				// Print to console if the reward is positive
				if (reward > 0) {
					std::cout << "ResetShotReward: " << reward << std::endl;
				}

				return reward;
			}

			if (player.isOnGround) {
				_tickCountWhenResetObtained.erase(it);
			}

			return 0.f;
		}
	};

	/**
	 * @brief A stateful reward for air dribbling while using air roll.
	 * It gives rewards for maintaining proximity to the ball, moving towards it,
	 * gaining height, staying at a target height, conserving boost, and using air roll effectively.
	 * It also gives a large bonus for each consecutive touch in the air.
	 */
	class AirDribbleWithRollRewardImpl : public Reward {
	public:
		// Configurable parameters
		float min_height = 326.0f;
		float max_distance = 250.0f;
		float proximity_weight = 3.0f;
		float velocity_weight = 3.0f;
		float height_weight = 1.0f;
		float touch_bonus = 3.0f;
		float boost_threshold = 20.0f; // Boost amount / 100
		float height_target = 1000.0f;
		float height_target_weight = 1.0f;
		float upward_bonus = 0.1f;
		float air_roll_weight = 0.5f;
		float speed_weight = 1.0f;
		float boost_efficiency_weight = 1.2f;

	private:
		// Per-player state
		bool last_ball_touch = false;
		int air_touch_streak = 0;
		float last_roll_input = 0.0f;
		int direction_changes = 0;

	public:
		virtual void Reset(const GameState& initialState) override {
			last_ball_touch = false;
			air_touch_streak = 0;
			last_roll_input = 0.0f;
			direction_changes = 0;
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Reset streak and state if on ground or conditions are not met
			bool is_airborne = !player.isOnGround && player.pos.z >= min_height && state.ball.pos.z >= min_height;
			float dist = player.pos.Dist(state.ball.pos);
			bool is_close = dist <= max_distance;

			if (!is_airborne || !is_close) {
				air_touch_streak = 0;
				last_ball_touch = player.ballTouchedStep;
				last_roll_input = 0.0f;
				direction_changes = 0;
				return 0.0f;
			}

			// --- Base Reward Components ---
			float prox_reward = (1.0f - (dist - CommonValues::BALL_RADIUS) / (max_distance - CommonValues::BALL_RADIUS)) * proximity_weight;

			Vec dir_to_ball = (state.ball.pos - player.pos).Normalized();
			float speed_towards_ball = player.vel.Dot(dir_to_ball);
			float vel_reward = (speed_towards_ball > 0) ? (speed_towards_ball / CommonValues::CAR_MAX_SPEED) * velocity_weight : 0;

			float height_reward = ((player.pos.z - min_height) / (CommonValues::CEILING_Z - min_height)) * height_weight;

			float height_diff = abs(player.pos.z - height_target);
			float height_target_reward = (1.0f - height_diff / height_target) * height_target_weight;

			float controlled_speed_reward = 0.0f;
			float player_speed = player.vel.Length();
			if (player_speed >= 800.0f && player_speed <= 1400.0f) {
				controlled_speed_reward = speed_weight;
			}
			else if (player_speed > 1400.0f) {
				controlled_speed_reward = -std::min(0.5f, (player_speed - 1400.0f) / 1000.0f) * speed_weight;
			}

			float boost_conservation_reward = (player.boost > 30) ? (powf(player.boost / 100.f, 0.7f) * boost_efficiency_weight) : 0;

			// --- Air Roll Reward ---
			float air_roll_reward = 0.0f;
			float current_roll_input = player.prevAction.roll;

			if (abs(current_roll_input) > 0.1f && abs(last_roll_input) > 0.1f) {
				if (current_roll_input * last_roll_input < 0) { // Sign change
					direction_changes += 1;
				}
			}
			if (direction_changes > 0) direction_changes = std::max(0, direction_changes - 1); // Decay
			last_roll_input = current_roll_input;

			bool is_wiggling = direction_changes > 4; // Check for excessive wiggling
			if (!is_wiggling && abs(current_roll_input) > 0.1f) {
				air_roll_reward = air_roll_weight;
			}

			// --- Combine Rewards ---
			float base_reward = prox_reward + vel_reward + height_reward + height_target_reward + controlled_speed_reward + boost_conservation_reward + air_roll_reward;
			float total_weight = proximity_weight + velocity_weight + height_weight + height_target_weight + speed_weight + boost_efficiency_weight + 1.0f;
			float final_reward = base_reward / total_weight;

			if (player.vel.z > 0) final_reward += upward_bonus;

			// --- Touch Bonus ---
			if (!last_ball_touch && player.ballTouchedStep) {
				air_touch_streak += 1;
				float touch_multiplier = 1.0f;
				if (player_speed >= 800.0f && player_speed <= 1400.0f) touch_multiplier *= 2.5f;
				if (player.boost > boost_threshold) touch_multiplier *= 1.8f;
				if (dist < CommonValues::BALL_RADIUS + 150.0f) touch_multiplier *= 1.4f;

				final_reward += touch_bonus * air_touch_streak * touch_multiplier;
			}

			last_ball_touch = player.ballTouchedStep;

			// Final scaling based on height
			final_reward *= std::max(0.1f, std::min(2.0f, state.ball.pos.z / 1000.0f));

			return final_reward;
		}
	};

	/**
	 * @brief Rewards the ball for having high velocity towards the opponent's goal.
	 */
	class SpeedBallToGoalReward : public Reward {
	public:
		bool ownGoal = false;
		SpeedBallToGoalReward(bool ownGoal = false) : ownGoal(ownGoal) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;

			Vec targetPos = targetOrangeGoal ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;

			Vec dirToGoal = (targetPos - state.ball.pos).Normalized();
			return state.ball.vel.Dot(dirToGoal) / CommonValues::BALL_MAX_SPEED;
		}
	};



	class EnergyReward : public Reward {
	public:
		const double GRAVITY = 650;
		const double MASS = 180;
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			auto max_energy = (MASS * GRAVITY * (CEILING_Z - 17.)) + (0.5 * MASS * (CAR_MAX_SPEED * CAR_MAX_SPEED));
			double energy = 0;
			double velocity = player.vel.Length();

			if (player.HasFlipOrJump()) {
				energy += 0.35 * MASS * (292 * 292);
			}
			if (player.HasFlipOrJump() and !player.isOnGround) {
				double dodge_impulse = (velocity <= 1700) ? (500 + (velocity / 17)) : (600 - (velocity - 1700));
				dodge_impulse = std::max(dodge_impulse - 25, 0.0);
				energy += 0.9 * 0.5 * MASS * (dodge_impulse * dodge_impulse);
				energy += 0.35 * MASS * 550. * 550.;
			}
			//height
			energy += MASS * GRAVITY * (player.pos.z - 17.) * 0.75; // fudge factor to reduce height
			//KE
			energy += 0.5 * MASS * velocity * velocity;
			//boost
			energy += 7.97e5 * player.boost;
			double norm_energy = player.isDemoed ? 0.0f : (energy / max_energy);
			return norm_energy;
		}
	};

	class KaiyoEnergyReward : public Reward {
	public:
		const double GRAVITY = 650;
		const double MASS = 180;
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			const auto max_energy = (MASS * GRAVITY * (CommonValues::CEILING_Z - 17.)) + (0.5 * MASS * (CommonValues::CAR_MAX_SPEED * CommonValues::CAR_MAX_SPEED));
			double energy = 0;

			if (player.HasFlipOrJump()) {
				energy += 0.35 * MASS * 292. * 292.;
			}

			if (player.HasFlipOrJump() && !player.isOnGround) {
				energy += 0.35 * MASS * 550. * 550.;
			}

			energy += MASS * GRAVITY * (player.pos.z - 17.) * 0.75;

			double velocity = player.vel.Length();
			energy += 0.5 * MASS * velocity * velocity;
			energy += 7.97e6 * player.boost;

			double norm_energy = player.isDemoed ? 0.0f : (energy / max_energy);

			return static_cast<float>(norm_energy);
		}
	};




	class AirdribbleSetupReward : public Reward {
	public:
		// Tunable parameters for the reward function
		const float MIN_BALL_HEIGHT = CommonValues::BALL_RADIUS + 15.f;
		const float MIN_UPWARD_VELOCITY = 100.f;
		const float MAX_DISTANCE_TO_BALL = 500.f;
		const float IDEAL_HEIGHT_DIFFERENCE = 150.f; // Bot should be this far below the ball
		const float MAX_REWARDED_POP_VEL = 1500.f;

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// We need the previous state to analyze the result of the touch.
			if (!player.prev || !player.ballTouchedStep) {
				return 0.f;
			}

			const auto& prevBallState = state.prev->ball;
			const auto& ballState = state.ball;
			const auto& playerState = player;

			// 1. The Pop-up: Ensure the ball is popped into the air with upward velocity.
			if (ballState.pos.z < MIN_BALL_HEIGHT || ballState.vel.z < MIN_UPWARD_VELOCITY) {
				return 0.f;
			}

			// Reward for imparting upward velocity on the ball.
			float popReward = RS_CLAMP(ballState.vel.z / MAX_REWARDED_POP_VEL, 0.f, 1.f);

			// 2. Player Follow-up: Ensure the player is also airborne and following the ball.
			if (playerState.isOnGround) {
				return 0.f;
			}

			// Reward for being close to the ball.
			float distToBall = playerState.pos.Dist(ballState.pos);
			if (distToBall > MAX_DISTANCE_TO_BALL) {
				return 0.f;
			}
			float proximityReward = 1.f - (distToBall / MAX_DISTANCE_TO_BALL);

			// Reward for having velocity directed towards the ball.
			Vec dirToBall = (ballState.pos - playerState.pos).Normalized();
			float velocityAlignmentReward = (playerState.vel.Normalized().Dot(dirToBall) + 1.f) / 2.f;

			// 3. Positioning: Reward for being underneath the ball.
			if (playerState.pos.z >= ballState.pos.z) {
				return 0.f;
			}

			// Use a Gaussian function to give max reward at an ideal height difference.
			float heightDiff = ballState.pos.z - playerState.pos.z;
			float underBallReward = exp(-pow(heightDiff - IDEAL_HEIGHT_DIFFERENCE, 2) / (2 * pow(50, 2)));

			// 4. Goal Alignment: Reward for setting up towards the opponent's goal.
			Vec opponentGoal = (player.team == Team::BLUE) ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;
			Vec ballToGoalDir = (opponentGoal - ballState.pos).Normalized();
			float goalAlignmentReward = (ballState.vel.Normalized().Dot(ballToGoalDir) + 1.f) / 2.f;

			// Combine all reward components.
			// By multiplying, we ensure that all conditions must be met to receive a reward.
			float totalReward =
				popReward *
				proximityReward *
				velocityAlignmentReward *
				underBallReward *
				goalAlignmentReward;

			if (isnan(totalReward) || isinf(totalReward)) {
				return 0.f;
			}

			return totalReward;
		}
	};

	class DoubleTapReward : public Reward {
	private:
		// Enum to track the state of a double tap attempt
		enum class AttemptState {
			IDLE,                   // Not currently attempting a double tap
			AWAITING_BACKBOARD_HIT, // Player has hit the ball towards the backboard
			AWAITING_SECOND_TOUCH,  // Ball has hit the backboard, waiting for player's follow-up
			AWAITING_GOAL           // Player has made the follow-up touch, waiting for a goal
		};

		// Struct to hold the state for each player's attempt
		struct DoubleTapAttempt {
			AttemptState state = AttemptState::IDLE;
			uint64_t last_update_tick = 0;
			uint64_t floor_hit_tick = 0;
		};

		std::vector<DoubleTapAttempt> player_attempts;
		bool debug;

	public:
		/**
		 * @brief Construct a new Double Tap Reward object.
		 * @param debug_mode If true, will print messages to the console for tracking double tap events.
		 */
		DoubleTapReward(bool debug_mode = false) : debug(debug_mode) {}

		// Called once when the environment is reset.
		virtual void Reset(const GameState& initialState) override {
			player_attempts.clear();
			player_attempts.resize(initialState.players.size());
		}

		// Called for each player every step to calculate their reward.
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev) return 0; // Need previous state for comparisons

			auto& attempt = player_attempts[player.index];

			// This is a safe way to get tick_time, even with tick skip.
			float tick_time = (state.lastTickCount > state.prev->lastTickCount) ?
				(state.deltaTime / (state.lastTickCount - state.prev->lastTickCount)) :
				(1.f / 120.f);

			// Timeout the attempt if too much time has passed (e.g., 5 seconds)
			if (attempt.state != AttemptState::IDLE && (state.lastTickCount - attempt.last_update_tick) * tick_time > 5.0f) {
				attempt = {}; // Reset state
			}

			// Invalidate attempt if another player touches the ball
			if (state.lastTouchCarID != -1 && state.lastTouchCarID != player.carId && attempt.state != AttemptState::IDLE) {
				attempt = {}; // Reset state
			}

			// State machine for the double tap attempt
			switch (attempt.state) {
			case AttemptState::IDLE: {
				if (player.ballTouchedStep) {
					// Heuristic: Is it a potential setup touch? Ball hit high and fast towards the opponent's backboard.
					bool is_blue_team = player.team == Team::BLUE;
					float ball_y_vel = state.ball.vel.y;
					float ball_z_pos = state.ball.pos.z;

					bool heading_to_opp_backboard = (is_blue_team && ball_y_vel > 1000) || (!is_blue_team && ball_y_vel < -1000);

					if (heading_to_opp_backboard && ball_z_pos > 500) {
						attempt.state = AttemptState::AWAITING_BACKBOARD_HIT;
						attempt.last_update_tick = state.lastTickCount;
					}
				}
				break;
			}

			case AttemptState::AWAITING_BACKBOARD_HIT: {
				// Check for a backboard rebound
				float ball_y_pos = state.ball.pos.y;
				if (abs(ball_y_pos) > CommonValues::BACK_WALL_Y - CommonValues::BALL_RADIUS * 1.5) {
					float prev_ball_y_vel = state.prev->ball.vel.y;
					float cur_ball_y_vel = state.ball.vel.y;

					// Check if y-velocity has flipped sign, indicating a bounce
					if (std::signbit(prev_ball_y_vel) != std::signbit(cur_ball_y_vel)) {
						if (debug) {
							std::cout << "DEBUG: Player " << player.carId << " got a backboard rebound!" << std::endl;
						}
						attempt.state = AttemptState::AWAITING_SECOND_TOUCH;
						attempt.last_update_tick = state.lastTickCount;
					}
				}
				break;
			}

			case AttemptState::AWAITING_SECOND_TOUCH: {
				if (player.ballTouchedStep) {
					// Player made the second touch
					attempt.state = AttemptState::AWAITING_GOAL;
					attempt.last_update_tick = state.lastTickCount;
					attempt.floor_hit_tick = 0; // Reset floor hit grace period timer
				}
				break;
			}

			case AttemptState::AWAITING_GOAL: {
				// Check if ball hits the floor to start the grace period timer
				if (attempt.floor_hit_tick == 0 && state.ball.pos.z < CommonValues::BALL_RADIUS + 15 && state.prev->ball.vel.z < 0) {
					attempt.floor_hit_tick = state.lastTickCount;
				}

				if (state.goalScored) {
					bool scored_on_correct_goal = (player.team != RS_TEAM_FROM_Y(state.ball.pos.y));

					if (scored_on_correct_goal && state.lastTouchCarID == player.carId) {

						bool within_grace_period = (attempt.floor_hit_tick == 0) ||
							((state.lastTickCount - attempt.floor_hit_tick) * tick_time <= 2.0f);

						if (within_grace_period) {
							if (debug) {
								std::cout << "DEBUG: Player " << player.carId << " scored a DOUBLE TAP!" << std::endl;
							}
							attempt = {}; // Reset state for next attempt
							return 1.0f;
						}
					}
				}
				break;
			}
			}

			return 0; // No reward this step
		}
	};

 class StrategicDribbleBumpReward : public Reward {
	private:
		struct PlayerState {
			bool wasDribbling = false;
			bool isDribbling = false;
			uint64_t ticksSinceBump = 999;
			uint64_t ticksSinceDribbleEnd = 999;
			bool hadStrategicBump = false;
			uint32_t bumpedOpponentId = 0;
			Vec ballPosAtBump;
			float dribbleProgress = 0.0f; // How close ball is to opponent goal when dribbling
			bool hadContactLastStep = false;
			uint32_t lastContactedOpponentId = 0;
		};

		std::unordered_map<uint32_t, PlayerState> playerStates;

		// Parameters
		float baseBumpReward;
		float demoBonus;
		float opponentNearGoalBonus;
		float dribbleProgressBonus;
		float savePenalty;
		float goalBonus;
		float dribbleTowardGoalBonus;
		float approachOpponentReward; // Reward for moving toward opponent
		float contactReward; // Reward for making contact with opponent
		float maxOpponentGoalDist;
		float maxDribbleDist;
		float minDribbleHeight;
		float maxDribbleHeight;
		float maxContactDist; // Distance to consider as "contact"
		uint64_t saveCheckWindow; // Ticks to check for save after bump

		bool IsDribbling(const Player& player, const GameState& state) const {
			if (!player.isOnGround) return false;
			if (state.ball.pos.z < minDribbleHeight || state.ball.pos.z > maxDribbleHeight) return false;
			float dist2D = player.pos.Dist2D(state.ball.pos);
			if (dist2D > maxDribbleDist) return false;
			return true;
		}

		bool IsOpponentNearGoal(const Player& opponent, Team opponentTeam) const {
			Vec opponentGoal = (opponentTeam == Team::BLUE)
				? CommonValues::BLUE_GOAL_CENTER
				: CommonValues::ORANGE_GOAL_CENTER;
			return opponent.pos.Dist2D(opponentGoal) < maxOpponentGoalDist;
		}

		bool IsDribblingTowardGoal(const Player& player, const GameState& state) const {
			Vec opponentGoal = (player.team == Team::BLUE)
				? CommonValues::ORANGE_GOAL_CENTER
				: CommonValues::BLUE_GOAL_CENTER;

			Vec toGoal = (opponentGoal - state.ball.pos).Normalized();
			Vec ballVel = state.ball.vel.Normalized();

			// Check if ball is moving toward opponent goal
			return toGoal.Dot(ballVel) > 0.3f;
		}

		bool IsBallMovingTowardOpponentGoal(const Player& player, const GameState& state) const {
			Vec opponentGoal = (player.team == Team::BLUE)
				? CommonValues::ORANGE_GOAL_CENTER
				: CommonValues::BLUE_GOAL_CENTER;

			Vec toGoal = opponentGoal - state.ball.pos;
			float toGoalDist = toGoal.Length();
			if (toGoalDist < 1e-6f) return false; // Ball is at goal

			Vec toGoalDir = toGoal / toGoalDist;

			float ballVelLen = state.ball.vel.Length();
			if (ballVelLen < 1e-6f) return false; // Ball not moving

			Vec ballVelDir = state.ball.vel / ballVelLen;

			// Check if ball velocity is toward opponent goal (dot product > 0 means moving toward)
			return toGoalDir.Dot(ballVelDir) > 0.0f;
		}

		float CalculateDribbleProgress(const Player& player, const GameState& state) const {
			Vec opponentGoal = (player.team == Team::BLUE)
				? CommonValues::ORANGE_GOAL_CENTER
				: CommonValues::BLUE_GOAL_CENTER;

			float distToGoal = state.ball.pos.Dist2D(opponentGoal);
			// Normalize: closer to goal = higher progress (max distance ~8000)
			return RS_MAX(0.0f, 1.0f - (distToGoal / 8000.0f));
		}

	public:
		StrategicDribbleBumpReward(
			float baseBumpReward = 5.0f,
			float demoBonus = 3.0f,
			float opponentNearGoalBonus = 4.0f,
			float dribbleProgressBonus = 2.0f,
			float savePenalty = -8.0f,
			float goalBonus = 15.0f,
			float dribbleTowardGoalBonus = 2.0f,
			float approachOpponentReward = 0.5f,
			float contactReward = 2.0f,
			float maxOpponentGoalDist = 2000.0f,
			float maxDribbleDist = 200.0f,
			float minDribbleHeight = 109.0f,
			float maxDribbleHeight = 250.0f,
			float maxContactDist = 300.0f,
			uint64_t saveCheckWindow = 180 // ~1.5 seconds at 120 tick rate
		) : baseBumpReward(baseBumpReward),
			demoBonus(demoBonus),
			opponentNearGoalBonus(opponentNearGoalBonus),
			dribbleProgressBonus(dribbleProgressBonus),
			savePenalty(savePenalty),
			goalBonus(goalBonus),
			dribbleTowardGoalBonus(dribbleTowardGoalBonus),
			approachOpponentReward(approachOpponentReward),
			contactReward(contactReward),
			maxOpponentGoalDist(maxOpponentGoalDist),
			maxDribbleDist(maxDribbleDist),
			minDribbleHeight(minDribbleHeight),
			maxDribbleHeight(maxDribbleHeight),
			maxContactDist(maxContactDist),
			saveCheckWindow(saveCheckWindow) {
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				playerStates[player.carId] = PlayerState();
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			auto& pState = playerStates[player.carId];
			float reward = 0.0f;

			// Update dribbling state
			bool currentlyDribbling = IsDribbling(player, state);
			pState.wasDribbling = pState.isDribbling;
			pState.isDribbling = currentlyDribbling;

			// Update timers
			if (pState.ticksSinceBump < 999) pState.ticksSinceBump++;
			if (!currentlyDribbling && pState.wasDribbling) {
				pState.ticksSinceDribbleEnd = 0;
			}
			else if (pState.ticksSinceDribbleEnd < 999) {
				pState.ticksSinceDribbleEnd++;
			}

			// Update dribble progress when dribbling
			if (currentlyDribbling) {
				pState.dribbleProgress = CalculateDribbleProgress(player, state);
			}

			// Check for bump/demo events (only trigger once per bump)
			bool justBumped = player.eventState.bump && !pState.hadStrategicBump;
			bool justDemoed = player.eventState.demo && !pState.hadStrategicBump;

			// Check if this is a strategic bump (dribbling toward goal)
			if ((justBumped || justDemoed) && !pState.hadStrategicBump) {
				bool wasDribblingRecently = pState.wasDribbling || pState.ticksSinceDribbleEnd < 60;
				bool dribblingTowardGoal = IsDribblingTowardGoal(player, state);

				if (wasDribblingRecently && dribblingTowardGoal) {
					// Find the closest opponent that was bumped (most likely target)
					const Player* bumpedOpponent = nullptr;
					float closestDist = FLT_MAX;

					for (const auto& opponent : state.players) {
						if (opponent.team != player.team && opponent.eventState.bumped) {
							float dist = player.pos.Dist(opponent.pos);
							if (dist < closestDist) {
								closestDist = dist;
								bumpedOpponent = &opponent;
							}
						}
					}

					if (bumpedOpponent) {
						pState.bumpedOpponentId = bumpedOpponent->carId;
						pState.ballPosAtBump = state.ball.pos;
						pState.hadStrategicBump = true;
						pState.ticksSinceBump = 0;

						// Base reward for strategic bump
						reward += baseBumpReward;

						// Demo bonus
						if (justDemoed) {
							reward += demoBonus;
						}

						// Opponent near goal bonus
						if (IsOpponentNearGoal(*bumpedOpponent, bumpedOpponent->team)) {
							reward += opponentNearGoalBonus;
						}

						// Dribble progress bonus (closer to goal = more reward)
						if (pState.dribbleProgress > 0.5f) {
							reward += dribbleProgressBonus * pState.dribbleProgress;
						}

						// Dribbling toward goal bonus
						reward += dribbleTowardGoalBonus;
					}
				}
			}

			// Check for save after strategic bump (penalty)
			if (pState.hadStrategicBump && pState.ticksSinceBump < saveCheckWindow) {
				// Check if opponent saved
				for (const auto& opponent : state.players) {
					if (opponent.carId == pState.bumpedOpponentId && opponent.eventState.save) {
						reward += savePenalty;
						pState.hadStrategicBump = false; // Reset after penalty
						pState.ticksSinceBump = 999;
					}
				}
			}

			// Goal bonus (if goal scored after strategic bump)
			if (pState.hadStrategicBump && pState.ticksSinceBump < saveCheckWindow * 2) {
				if (state.goalScored) {
					// Check if our team scored
					for (const auto& p : state.players) {
						if (p.team == player.team && p.eventState.goal) {
							reward += goalBonus;
							pState.hadStrategicBump = false; // Reset after goal
							pState.ticksSinceBump = 999;
							break;
						}
					}
				}
			}

			// Reset strategic bump state after window expires
			if (pState.ticksSinceBump >= saveCheckWindow * 2) {
				pState.hadStrategicBump = false;
			}

			// Small reward for maintaining dribble toward goal
			if (currentlyDribbling && IsDribblingTowardGoal(player, state)) {
				reward += 0.1f * pState.dribbleProgress;
			}

			// Check if ball is moving toward opponent goal - REQUIRED for approach/contact rewards
			bool ballMovingTowardGoal = IsBallMovingTowardOpponentGoal(player, state);

			// If ball is not moving toward opponent goal, skip approach/contact rewards
			if (!ballMovingTowardGoal) {
				// Update contact state
				if (!player.eventState.bump && !player.eventState.demo) {
					// Only reset if we're not in close proximity to any opponent
					bool stillNearOpponent = false;
					for (const auto& opponent : state.players) {
						if (opponent.team != player.team) {
							if (player.pos.Dist(opponent.pos) < maxContactDist * 1.5f) {
								stillNearOpponent = true;
								break;
							}
						}
					}
					if (!stillNearOpponent) {
						pState.hadContactLastStep = false;
					}
				}
				return reward; // Return early - no approach/contact rewards if ball not moving toward goal
			}

			// Reward for approaching opponents and detect contact
			for (const auto& opponent : state.players) {
				if (opponent.team == player.team) continue;

				float distToOpponent = player.pos.Dist(opponent.pos);

				// Check if moving toward opponent
				Vec toOpponentVec = opponent.pos - player.pos;
				float toOpponentDist = toOpponentVec.Length();
				if (toOpponentDist < 1e-6f) continue; // Skip if cars are on top of each other

				Vec toOpponent = toOpponentVec / toOpponentDist;

				float playerVelLen = player.vel.Length();
				if (playerVelLen < 1e-6f) continue; // Skip if not moving

				Vec playerVelDir = player.vel / playerVelLen;
				float velTowardOpponent = player.vel.Dot(toOpponent);

				// Reward for moving toward opponent (scaled by speed and proximity)
				if (velTowardOpponent > 0.0f && distToOpponent < 2000.0f) {
					float approachReward = approachOpponentReward * (velTowardOpponent / CommonValues::CAR_MAX_SPEED);
					// Scale by proximity (closer = more reward)
					float proximityScale = RS_MAX(0.0f, 1.0f - (distToOpponent / 2000.0f));
					reward += approachReward * proximityScale;
				}

				// Detect contact (either through bump event or close proximity)
				bool madeContact = false;

				// Check for bump/demo events (explicit contact)
				// Only count if player initiated the contact (player has bump/demo event)
				if ((player.eventState.bump || player.eventState.demo) &&
					(opponent.eventState.bumped || opponent.eventState.demoed)) {
					// Verify this is the opponent we contacted
					if (distToOpponent < maxContactDist * 2.0f) {
						madeContact = true;
						pState.lastContactedOpponentId = opponent.carId;
					}
				}

				// Also detect contact through proximity (cars very close together)
				// This catches cases where cars make contact but might not trigger bump events immediately
				if (!madeContact && distToOpponent < maxContactDist) {
					// Check if we're moving toward each other (collision likely)
					float opponentVelLen = opponent.vel.Length();
					Vec opponentVelDir = (opponentVelLen > 1e-6f) ? (opponent.vel / opponentVelLen) : Vec(0, 0, 0);
					float relativeSpeed = (player.vel - opponent.vel).Length();

					// If cars are close and moving toward each other, consider it contact
					if (relativeSpeed > 500.0f && toOpponent.Dot(playerVelDir) > 0.3f) {
						// Only reward if we haven't already rewarded for this contact
						if (!pState.hadContactLastStep || pState.lastContactedOpponentId != opponent.carId) {
							madeContact = true;
							pState.lastContactedOpponentId = opponent.carId;
						}
					}
				}

				// Reward for making contact (only once per contact)
				if (madeContact && (!pState.hadContactLastStep || pState.lastContactedOpponentId != opponent.carId)) {
					reward += contactReward;

					// Bonus if contact happened while dribbling toward goal
					if (currentlyDribbling && IsDribblingTowardGoal(player, state)) {
						reward += contactReward * 0.5f;
					}

					// Bonus if opponent is near their goal
					if (IsOpponentNearGoal(opponent, opponent.team)) {
						reward += contactReward * 0.5f;
					}

					// Mark that we had contact this step
					pState.hadContactLastStep = true;
				}
			}

			// Update contact state (reset if no contact this step)
			if (!player.eventState.bump && !player.eventState.demo) {
				// Only reset if we're not in close proximity to any opponent
				bool stillNearOpponent = false;
				for (const auto& opponent : state.players) {
					if (opponent.team != player.team) {
						if (player.pos.Dist(opponent.pos) < maxContactDist * 1.5f) {
							stillNearOpponent = true;
							break;
						}
					}
				}
				if (!stillNearOpponent) {
					pState.hadContactLastStep = false;
				}
			}

			return reward;
		}
};

	class TeamSpacingReward : public Reward {
	public:
		float minSpacing;

		// minSpacing: Minimum distance between teammates (in Rocket League units)
		// Players get full reward when >= minSpacing apart, penalty when too close
		TeamSpacingReward(float minSpacing = 1500.0f) : minSpacing(minSpacing) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			float totalReward = 0.0f;
			int teammateCount = 0;

			// Check spacing with all teammates
			for (const Player& teammate : state.players) {
				// Skip self and opponents
				if (teammate.carId == player.carId || teammate.team != player.team)
					continue;

				teammateCount++;
				float distance = (player.pos - teammate.pos).Length();

				if (distance >= minSpacing) {
					// Good spacing - full reward
					totalReward += 1.0f;
				}
				else {
					// Too close - linear penalty based on how close they are
					float ratio = distance / minSpacing;  // 0 to 1
					totalReward += ratio;  // Linear reward from 0 to 1
				}
			}

			// Return average reward across all teammates (0 if no teammates)
			return teammateCount > 0 ? totalReward / teammateCount : 0.0f;
		}
	};

	class TeammateBumpPenaltyReward : public Reward {
	private:
		std::unordered_map<int, uint32_t> previousContactCarID;
		std::unordered_map<int, float> previousContactTimer;

	public:
		TeammateBumpPenaltyReward() {}

		virtual void Reset(const GameState& initialState) override {
			previousContactCarID.clear();
			previousContactTimer.clear();
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			float reward = 0.0f;

			uint32_t currentContactCarID = player.carContact.otherCarID;
			float currentContactTimer = player.carContact.cooldownTimer;

			auto prevCarIDIter = previousContactCarID.find(player.carId);
			auto prevTimerIter = previousContactTimer.find(player.carId);

			bool hadPreviousState = (prevCarIDIter != previousContactCarID.end() &&
				prevTimerIter != previousContactTimer.end());

			if (currentContactTimer > 0 && currentContactCarID != 0) {
				bool isNewBump = false;

				if (!hadPreviousState) {
					isNewBump = true;
				}
				else {
					float prevTimer = prevTimerIter->second;
					uint32_t prevCarID = prevCarIDIter->second;

					isNewBump = (currentContactTimer > prevTimer) ||
						(currentContactCarID != prevCarID && prevTimer <= 0);
				}

				if (isNewBump) {
					for (const auto& otherPlayer : state.players) {
						if (otherPlayer.carId == currentContactCarID) {
							if (otherPlayer.team == player.team) {
								reward -= 1.0f;
							}
							break;
						}
					}
				}
			}

			previousContactCarID[player.carId] = currentContactCarID;
			previousContactTimer[player.carId] = currentContactTimer;

			return reward;
		}
	};



	class RaptorTeamSpacingReward : public Reward {
	public:
		double minSpacing;

		RaptorTeamSpacingReward(double minSpacing = 1550.f)
			: minSpacing(std::clamp(minSpacing, 0.0000001, std::numeric_limits<double>::infinity())) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			double reward = 0.f;
			for (auto _player : state.players) {
				if ((_player.team == player.team) && (_player.carId != player.carId) && !player.isDemoed && !_player.isDemoed) {
					double separation = player.pos.Dist(_player.pos);
					if (separation < minSpacing) {
						reward -= 1 - (separation / minSpacing);
					}
				}
			}
			return reward;
		}
	};



	class DribbleReward : public Reward {
	public:
		float minBallHeight;
		float maxBallHeight;
		float maxDistance;
		float coeff;

		// Based on SwiftGroundDribbleReward - rewards speed matching between player and ball during dribbles
		DribbleReward(float minBallHeight = 109.0f, float maxBallHeight = 180.0f,
			float maxDistance = 197.0f, float coeff = 2.0f)
			: minBallHeight(minBallHeight), maxBallHeight(maxBallHeight),
			maxDistance(maxDistance), coeff(coeff) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Check all dribbling conditions
			if (!player.isOnGround) return 0.0f;
			if (state.ball.pos.z < minBallHeight || state.ball.pos.z > maxBallHeight) return 0.0f;
			if ((player.pos - state.ball.pos).Length() >= maxDistance) return 0.0f;

			// Calculate speed reward based on player-ball speed matching
			float playerSpeed = player.vel.Length();
			float ballSpeed = state.ball.vel.Length();
			float playerSpeedNormalized = playerSpeed / CommonValues::CAR_MAX_SPEED;
			float inverseDifference = 1.0f - abs(playerSpeed - ballSpeed);
			float twoSum = playerSpeed + ballSpeed;

			// Avoid division by zero
			if (twoSum == 0.0f) return 0.0f;

			float speedReward = playerSpeedNormalized + coeff * (inverseDifference / twoSum);
			return speedReward;
		}
	};

	class FlickReward : public Reward {
	public:
		float minFlickSpeed;

		// minFlickSpeed: Minimum ball speed to count as a flick
		FlickReward(float minFlickSpeed = 800.0f) : minFlickSpeed(minFlickSpeed) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev || !player.ballTouchedStep)
				return 0.0f;

			// Check if this was a flick (ball was low, now going fast and upward)
			bool ballWasLow = state.prev->ball.pos.z < 200.0f;
			bool ballNowFast = state.ball.vel.Length() > minFlickSpeed;
			bool ballGoingUp = state.ball.vel.z > 200.0f;

			// Player should have been close to ball when touching
			float prevDistance = (player.pos - state.prev->ball.pos).Length();
			bool playerWasClose = prevDistance < 200.0f;

			if (ballWasLow && ballNowFast && ballGoingUp && playerWasClose) {
				// Reward based on ball speed after flick
				float speedRatio = RS_MIN(1.0f, state.ball.vel.Length() / 2000.0f);
				return speedRatio;
			}

			return 0.0f;
		}
	};

	class KickoffProximityReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Check if ball is at kickoff position (0, 0, z)
			if (abs(state.ball.pos.x) > 1.0f || abs(state.ball.pos.y) > 1.0f) {
				return 0.0f; // Not a kickoff situation
			}

			// Calculate player's distance to ball
			float playerDistToBall = (player.pos - state.ball.pos).Length();

			// Find closest opponent distance to ball
			float closestOpponentDist = FLT_MAX;
			bool foundOpponent = false;

			for (const Player& opponent : state.players) {
				// Skip teammates and self
				if (opponent.team == player.team || opponent.carId == player.carId) {
					continue;
				}

				foundOpponent = true;
				float opponentDistToBall = (opponent.pos - state.ball.pos).Length();

				if (opponentDistToBall < closestOpponentDist) {
					closestOpponentDist = opponentDistToBall;
				}
			}

			// If no opponents found, return neutral
			if (!foundOpponent) {
				return 0.0f;
			}

			// Return 1 if player is closer, -1 if opponent is closer
			return (playerDistToBall < closestOpponentDist) ? 1.0f : -1.0f;
		}
	};

	class HyperNoStackReward : public Reward {
	private:
		struct PlayerState {
			bool wasOnWall = false;
			bool dashUsed = false;
			bool hadFlipBefore = false;
			bool wasFlipping = false;
			Vec prevVel = Vec(0, 0, 0);
			// int lastDemoCount = 0; // Removed: Not supported by current Player struct
		};

		std::map<uint32_t, PlayerState> playerStates;

		// --- Configuration ---
		float wallHeightThreshold;

		// Base Rewards
		float dashRewardBase;
		float resetRewardBase;
		float wavedashRewardBase;
		float zapDashBaseReward;
		// float demoReward; // Removed temporarily to fix build

		// Quality Scaling
		float accelerationScalar;

		// Penalties
		float wallStayPenalty;
		float supersonicBoostPenalty;

		// Zap Dash Config
		float zapMinSpeedGain;
		float zapMinNoseDown;
		float zapMinFwdDot;

		bool debug;

	public:
		HyperNoStackReward(
			// Base Values
			float dashRewardBase = 12.0f,
			float resetRewardBase = 16.0f,
			float wavedashRewardBase = 10.0f,
			float zapDashBaseReward = 15.0f,
			// float demoReward = 30.0f, 
			// Scaling
			float accelerationScalar = 1.0f,
			// Penalties
			float wallStayPenalty = -0.2f,
			float supersonicBoostPenalty = -0.1f,
			// Debug
			bool debug = false
		) :
			wallHeightThreshold(100.0f),
			dashRewardBase(dashRewardBase), resetRewardBase(resetRewardBase),
			wavedashRewardBase(wavedashRewardBase), zapDashBaseReward(zapDashBaseReward),
			// demoReward(demoReward),
			accelerationScalar(accelerationScalar),
			wallStayPenalty(wallStayPenalty), supersonicBoostPenalty(supersonicBoostPenalty),
			zapMinSpeedGain(500.0f), zapMinNoseDown(0.5f), zapMinFwdDot(0.7f),
			debug(debug)
		{
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			for (const auto& player : initialState.players) {
				InitState(player);
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev) return 0.0f;

			if (playerStates.find(player.carId) == playerStates.end()) {
				InitState(player);
			}
			PlayerState& st = playerStates[player.carId];

			float totalReward = 0.0f;
			float dt = 1.0f / 120.0f;

			// --- PRE-CALCULATIONS ---
			bool isOnWall = IsOnWall(player.pos);
			bool hasFlip = player.HasFlipOrJump();

			// Calculate Acceleration Quality
			float acceleration = (player.vel - st.prevVel).Length() / dt;
			float normAccel = std::min(acceleration / 5000.0f, 2.0f);
			float qualityMult = 1.0f + (normAccel * accelerationScalar);

			// =========================================================
			// MECHANIC DETECTION
			// =========================================================

			// 1. DEMOLITION (Disabled to fix build error)
			/*
			if (player.match_demolitions > st.lastDemoCount) {
				totalReward += demoReward * qualityMult;
				if (debug) printf("ID:%d | DEMO | Reward: %.2f\n", player.carId, demoReward * qualityMult);
			}
			*/

			// 2. ZAP DASH
			float zapRatio = CheckZapDash(player);
			if (zapRatio > 0.0f) {
				float r = zapDashBaseReward * zapRatio * qualityMult;
				totalReward += r;
				if (debug) printf("ID:%d | ZapDash | Reward: %.2f\n", player.carId, r);
			}

			// 3. WALL DASH (LAUNCH)
			if (st.wasOnWall && st.hadFlipBefore && !hasFlip && !isOnWall) {
				st.dashUsed = true;
				float r = dashRewardBase * qualityMult;
				totalReward += r;
				if (debug) printf("ID:%d | WallDash_Launch | Reward: %.2f\n", player.carId, r);
			}

			// 4. WALL DASH (RESET)
			if (st.dashUsed && isOnWall && hasFlip) {
				st.dashUsed = false; // Cycle complete
				float r = resetRewardBase * qualityMult;
				totalReward += r;
				if (debug) printf("ID:%d | WallDash_Reset | Reward: %.2f\n", player.carId, r);
			}

			// 5. WAVEDASH
			const Player* prevPlayer = nullptr;
			for (const auto& p : state.prev->players) { if (p.carId == player.carId) { prevPlayer = &p; break; } }

			bool justLanded = player.isOnGround && prevPlayer && !prevPlayer->isOnGround;
			if (justLanded && (player.isFlipping || st.wasFlipping)) {
				float r = wavedashRewardBase * qualityMult;
				totalReward += r;
				if (debug) printf("ID:%d | Wavedash | Reward: %.2f\n", player.carId, r);
			}

			// =========================================================
			// PENALTIES
			// =========================================================

			// 1. Supersonic Boost Waste
			if (prevPlayer && player.isSupersonic && player.boost < prevPlayer->boost) {
				totalReward += supersonicBoostPenalty;
			}

			// 2. Conditional Wall Stay Penalty
			if (isOnWall && st.dashUsed && player.vel.Length() < 600.0f) {
				totalReward += wallStayPenalty;
			}

			// =========================================================
			// STATE UPDATES
			// =========================================================
			st.wasOnWall = isOnWall;
			st.hadFlipBefore = hasFlip;
			st.wasFlipping = player.isFlipping;
			st.prevVel = player.vel;
			// st.lastDemoCount = player.match_demolitions; // Disabled

			if (player.isOnGround) {
				st.dashUsed = false;
			}

			return totalReward;
		}

	private:
		void InitState(const Player& p) {
			PlayerState newState;
			newState.prevVel = p.vel;
			newState.hadFlipBefore = p.HasFlipOrJump();
			// newState.lastDemoCount = p.match_demolitions; // Disabled
			playerStates[p.carId] = newState;
		}

		float CheckZapDash(const Player& current) {
			if (!current.prev || !current.prev->prev || !current.prev->prev->prev) return 0.0f;

			const Player* p0 = &current;
			const Player* p1 = current.prev;
			const Player* p2 = p1->prev;
			const Player* p3 = p2->prev;

			bool wasAirborneT3 = !p3->isOnGround;
			bool landedT2 = wasAirborneT3 && p2->isOnGround;
			bool noseDown = p2->rotMat.forward.z < -zapMinNoseDown;
			bool wasMovingDown = p2->vel.z < -200.0f;
			bool executingFlip = p0->isFlipping;

			if (landedT2 && noseDown && wasMovingDown && executingFlip) {
				Vec carFwd = p1->rotMat.forward;
				Vec velDir = p0->vel.Normalized();
				if (carFwd.Dot(velDir) > zapMinFwdDot) {
					float gain = p0->vel.Length() - p2->vel.Length();
					if (gain > zapMinSpeedGain) {
						float bonus = (p0->vel.Length() > 2200.f) ? 1.5f : 1.0f;
						return (gain / 1000.0f) * bonus;
					}
				}
			}
			return 0.0f;
		}

		bool IsOnWall(const Vec& pos) {
			if (pos.z < wallHeightThreshold) return false;
			if (std::abs(std::abs(pos.x) - 4096.0f) < 150.0f) return true;
			if (std::abs(std::abs(pos.y) - 5120.0f) < 150.0f) return true;
			return false;
		}
	};




	class KickoffProximityReward2v2Enhanced : public Reward {
	public:
		float goerReward = 1.2f;           // Increased base reward for goer
		float cheaterReward = 0.6f;        // Base reward for strategic cheater
		float dynamicWeight = 0.3f;        // Weight for dynamic adjustments
		float rotationPrepWeight = 0.2f;   // Weight for rotation preparation

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Enhanced kickoff detection - more robust
			if (!IsKickoffActive(state)) return 0.f;

			float playerDistToBall = (player.pos - state.ball.pos).Length();

			// Enhanced team analysis
			TeamAnalysis analysis = AnalyzeTeamState(player, state);
			if (!analysis.hasTeammate) return 0.f;

			// Dynamic role assignment with multiple factors
			PlayerRole role = DeterminePlayerRole(player, analysis, state);

			if (role == PlayerRole::GOER) {
				return CalculateGoerReward(player, analysis, state);
			}
			else {
				return CalculateCheaterReward(player, analysis, state);
			}
		}

	private:
		enum class PlayerRole { GOER, CHEATER };

		struct TeamAnalysis {
			bool hasTeammate = false;
			const Player* teammate = nullptr;
			float teammateDistToBall = 0.f;
			float closestOpponentDist = FLT_MAX;
			float secondOpponentDist = FLT_MAX;
			Vec opponentCenterOfMass = Vec(0.f, 0.f, 0.f);  // Fixed: explicit float construction
			float avgOpponentSpeed = 0.f;
		};

		bool IsKickoffActive(const GameState& state) {
			// More sophisticated kickoff detection
			float ballSpeed = state.ball.vel.Length();
			float ballHeight = state.ball.pos.z;
			Vec ballPos2D = Vec(state.ball.pos.x, state.ball.pos.y, 0.f);  // Fixed: explicit float for z

			return (ballSpeed < 2.f &&
				ballHeight < 150.f &&
				ballPos2D.Length() < 50.f);
		}

		TeamAnalysis AnalyzeTeamState(const Player& player, const GameState& state) {
			TeamAnalysis analysis;
			int opponentCount = 0;
			float totalOpponentSpeed = 0.f;

			for (const auto& p : state.players) {
				if (p.team == player.team && p.carId != player.carId) {
					analysis.teammate = &p;
					analysis.hasTeammate = true;
					analysis.teammateDistToBall = (p.pos - state.ball.pos).Length();
				}
				else if (p.team != player.team) {
					float opponentDist = (p.pos - state.ball.pos).Length();
					totalOpponentSpeed += p.vel.Length();
					opponentCount++;

					if (opponentDist < analysis.closestOpponentDist) {
						analysis.secondOpponentDist = analysis.closestOpponentDist;
						analysis.closestOpponentDist = opponentDist;
					}
					else if (opponentDist < analysis.secondOpponentDist) {
						analysis.secondOpponentDist = opponentDist;
					}

					analysis.opponentCenterOfMass = analysis.opponentCenterOfMass + p.pos;  // Fixed: use + instead of +=
				}
			}

			if (opponentCount > 0) {
				float countFloat = (float)opponentCount;  // Fixed: explicit cast
				analysis.opponentCenterOfMass = analysis.opponentCenterOfMass / countFloat;  // Fixed: explicit division
				analysis.avgOpponentSpeed = totalOpponentSpeed / countFloat;
			}

			return analysis;
		}

		PlayerRole DeterminePlayerRole(const Player& player, const TeamAnalysis& analysis, const GameState& state) {
			float playerDistToBall = (player.pos - state.ball.pos).Length();

			// Factor 1: Distance to ball (40% weight)
			float distanceScore = (playerDistToBall < analysis.teammateDistToBall) ? 0.4f : 0.f;

			// Factor 2: Speed toward ball (30% weight) - Fixed type issues
			Vec playerToBall = (state.ball.pos - player.pos).Normalized();
			Vec teammateToBall = (state.ball.pos - analysis.teammate->pos).Normalized();
			float playerVelToBall = player.vel.Dot(playerToBall);
			float teammateVelToBall = analysis.teammate->vel.Dot(teammateToBall);
			float speedScore = (playerVelToBall > teammateVelToBall) ? 0.3f : 0.f;

			// Factor 3: Boost level consideration (20% weight)
			float boostScore = (player.boost > analysis.teammate->boost + 10.f) ? 0.2f : 0.f;

			// Factor 4: Spawn position advantage (10% weight)
			float spawnScore = CalculateSpawnAdvantage(player, *analysis.teammate, state) * 0.1f;

			float totalScore = distanceScore + speedScore + boostScore + spawnScore;

			return (totalScore >= 0.5f) ? PlayerRole::GOER : PlayerRole::CHEATER;
		}

		float CalculateSpawnAdvantage(const Player& player, const Player& teammate, const GameState& state) {
			// Advantage based on diagonal vs straight kickoff positioning
			float playerAngleToBall = atan2f(player.pos.y - state.ball.pos.y, player.pos.x - state.ball.pos.x);
			float teammateAngleToBall = atan2f(teammate.pos.y - state.ball.pos.y, teammate.pos.x - state.ball.pos.x);

			float angleDiff = fabsf(playerAngleToBall - teammateAngleToBall);

			// Diagonal spawns (corner positions) have advantage for going
			return (angleDiff > (3.14159f / 3.f)) ? 1.f : 0.f;  // Fixed: use explicit float for PI
		}

		float CalculateGoerReward(const Player& player, const TeamAnalysis& analysis, const GameState& state) {
			float playerDistToBall = (player.pos - state.ball.pos).Length();

			// Base reward for being closer than opponents
			float baseReward = (playerDistToBall < analysis.closestOpponentDist) ? goerReward : -goerReward * 0.5f;

			// Speed differential bonus - Fixed type issues
			Vec playerToBall = (state.ball.pos - player.pos).Normalized();
			float playerVelToBall = player.vel.Dot(playerToBall);
			float speedBonus = RS_CLAMP(playerVelToBall / 2300.f, -0.3f, 0.3f); // Max car speed ~2300

			// Boost usage efficiency (penalize waste, reward conservation for crucial moments)
			float boostEfficiency = 0.f;
			if (player.boost > 50.f && playerDistToBall > 1000.f) {
				boostEfficiency = 0.1f; // Good boost management
			}
			else if (player.boost < 20.f && playerDistToBall > 800.f) {
				boostEfficiency = -0.15f; // Poor boost management
			}

			// Angle approach bonus (reward straight-line approaches)
			Vec toBall = (state.ball.pos - player.pos).Normalized();
			Vec velocity = player.vel.Normalized();
			float approachAngle = toBall.Dot(velocity);
			float angleBonus = RS_MAX(0.f, approachAngle) * 0.2f;

			return RS_CLAMP(baseReward + speedBonus + boostEfficiency + angleBonus, -1.5f, 1.5f);
		}

		float CalculateCheaterReward(const Player& player, const TeamAnalysis& analysis, const GameState& state) {
			Vec ownGoal = (player.team == Team::BLUE) ?
				CommonValues::BLUE_GOAL_BACK : CommonValues::ORANGE_GOAL_BACK;

			// Dynamic ideal position based on game state
			Vec idealPos = CalculateDynamicIdealPosition(player, analysis, state, ownGoal);
			float distToIdeal = (player.pos - idealPos).Length();

			// COMPONENT 1: Dynamic positioning (40% weight)
			float positioningReward = CalculatePositioningReward(player, idealPos, distToIdeal);

			// COMPONENT 2: Strategic boost management (25% weight)
			float boostReward = CalculateStrategicBoostReward(player, state, analysis) * 0.25f;

			// COMPONENT 3: Rotation preparation (20% weight)
			float rotationReward = CalculateRotationPreparation(player, analysis, state) * rotationPrepWeight;

			// COMPONENT 4: Opponent awareness (10% weight)
			float awarenessReward = CalculateOpponentAwareness(player, analysis, state) * 0.1f;

			// COMPONENT 5: Anti-camping with dynamic threshold (5% weight)
			float campingPenalty = CalculateDynamicCampingPenalty(player, ownGoal, state) * 0.05f;

			float totalReward = positioningReward + boostReward + rotationReward + awarenessReward + campingPenalty;

			return RS_CLAMP(totalReward, -0.8f, 0.8f);
		}

		Vec CalculateDynamicIdealPosition(const Player& player, const TeamAnalysis& analysis,
			const GameState& state, const Vec& ownGoal) {
			Vec fieldCenter = Vec(0.f, 0.f, 100.f);  // Fixed: explicit floats

			// Base position: 65% toward center from goal (slightly more aggressive than original)
			Vec centerMultiplied = Vec(fieldCenter.x * 1.3f, fieldCenter.y * 1.3f, fieldCenter.z * 1.3f);  // Fixed: manual multiplication
			Vec baseIdeal = (ownGoal + centerMultiplied) * 0.5f;

			// Adjust based on opponent positioning
			Vec opponentThreatVector = (analysis.opponentCenterOfMass - ownGoal).Normalized();
			opponentThreatVector = Vec(opponentThreatVector.x * 200.f, opponentThreatVector.y * 200.f, opponentThreatVector.z * 200.f);  // Fixed: manual scaling

			// Adjust based on teammate position (create optimal spacing)
			Vec teammateOffset = Vec(0.f, 0.f, 0.f);  // Fixed: explicit floats
			if (analysis.teammate) {
				Vec teammatePos = analysis.teammate->pos;
				float teammateDistFromCenter = (teammatePos - fieldCenter).Length();

				// If teammate is far from center, position closer to support
				if (teammateDistFromCenter > 1500.f) {
					Vec direction = (teammatePos - baseIdeal).Normalized();
					teammateOffset = Vec(direction.x * 300.f, direction.y * 300.f, direction.z * 300.f);  // Fixed: manual scaling
				}
			}

			// Final position with adjustments - Fixed: manual scaling
			Vec threatAdjustment = Vec(opponentThreatVector.x * 0.3f, opponentThreatVector.y * 0.3f, opponentThreatVector.z * 0.3f);
			Vec teammateAdjustment = Vec(teammateOffset.x * 0.2f, teammateOffset.y * 0.2f, teammateOffset.z * 0.2f);
			Vec adjustedIdeal = baseIdeal + threatAdjustment + teammateAdjustment;

			// Clamp to reasonable field boundaries
			adjustedIdeal.x = RS_CLAMP(adjustedIdeal.x, -3000.f, 3000.f);
			adjustedIdeal.y = RS_CLAMP(adjustedIdeal.y, -4000.f, 4000.f);
			adjustedIdeal.z = RS_MAX(adjustedIdeal.z, 17.f); // Ground level

			return adjustedIdeal;
		}

		float CalculatePositioningReward(const Player& player, const Vec& idealPos, float distToIdeal) {
			float optimalRadius = 600.f;
			float acceptableRadius = 1200.f;
			float maxRadius = 2000.f;

			if (distToIdeal <= optimalRadius) {
				// Excellent positioning
				return 0.5f * (1.f - (distToIdeal / optimalRadius));
			}
			else if (distToIdeal <= acceptableRadius) {
				// Good positioning with gradual falloff
				float ratio = (distToIdeal - optimalRadius) / (acceptableRadius - optimalRadius);
				return 0.5f * (1.f - ratio) * 0.7f;
			}
			else if (distToIdeal <= maxRadius) {
				// Poor but acceptable positioning
				float ratio = (distToIdeal - acceptableRadius) / (maxRadius - acceptableRadius);
				return -0.1f * ratio;
			}
			else {
				// Very poor positioning
				return -0.3f;
			}
		}

		float CalculateStrategicBoostReward(const Player& player, const GameState& state, const TeamAnalysis& analysis) {
			// Find strategically important boost pads
			float bestBoostValue = 0.f;

			for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
				const Vec& boostPos = CommonValues::BOOST_LOCATIONS[i];

				if (boostPos.z > 72.0f) { // Large boost pad
					float distToBoost = (player.pos - boostPos).Length();

					// Value boost based on multiple factors
					float accessibility = 1.f - RS_CLAMP(distToBoost / 1500.f, 0.f, 1.f);
					float strategicValue = CalculateBoostStrategicValue(boostPos, analysis, state);
					float denyValue = CalculateBoostDenialValue(boostPos, analysis);

					float totalValue = accessibility * (strategicValue + denyValue);
					bestBoostValue = RS_MAX(bestBoostValue, totalValue);
				}
			}

			// Boost level consideration
			float boostLevelFactor = 1.f;
			if (player.boost < 30.f) {
				boostLevelFactor = 1.5f; // More urgent need for boost
			}
			else if (player.boost > 80.f) {
				boostLevelFactor = 0.5f; // Less urgent need
			}

			return bestBoostValue * boostLevelFactor;
		}

		float CalculateBoostStrategicValue(const Vec& boostPos, const TeamAnalysis& analysis, const GameState& state) {
			// Boost pads closer to expected ball trajectory are more valuable
			Vec ballToBoost = (boostPos - state.ball.pos);
			float distToBall = ballToBoost.Length();

			// Corner boosts are generally more valuable for rotations
			bool isCornerBoost = (fabsf(boostPos.x) > 2500.f && fabsf(boostPos.y) > 3500.f);

			float baseValue = isCornerBoost ? 0.8f : 0.6f;
			float proximityValue = 1.f - RS_CLAMP(distToBall / 3000.f, 0.f, 1.f);

			return baseValue * (0.3f + proximityValue * 0.7f);
		}

		float CalculateBoostDenialValue(const Vec& boostPos, const TeamAnalysis& analysis) {
			// Value boost based on opponent accessibility
			float opponentDistToBoost = (analysis.opponentCenterOfMass - boostPos).Length();

			return RS_CLAMP(1.f - (opponentDistToBoost / 2000.f), 0.f, 0.3f);
		}

		float CalculateRotationPreparation(const Player& player, const TeamAnalysis& analysis, const GameState& state) {
			if (!analysis.teammate) return 0.f;

			// Reward positioning that allows quick transition to support teammate
			Vec teammatePos = analysis.teammate->pos;
			Vec supportPosition = CalculateOptimalSupportPosition(teammatePos, state.ball.pos, player.team);

			float distToSupport = (player.pos - supportPosition).Length();
			float supportReadiness = 1.f - RS_CLAMP(distToSupport / 1000.f, 0.f, 1.f);

			// Velocity alignment for quick rotation
			Vec toSupport = (supportPosition - player.pos).Normalized();
			float velocityAlignment = RS_MAX(0.f, player.vel.Normalized().Dot(toSupport));

			return (supportReadiness * 0.7f + velocityAlignment * 0.3f);
		}

		Vec CalculateOptimalSupportPosition(const Vec& teammatePos, const Vec& ballPos, Team team) {
			Vec ownGoal = (team == Team::BLUE) ?
				CommonValues::BLUE_GOAL_BACK : CommonValues::ORANGE_GOAL_BACK;

			// Position that forms good triangle with teammate and goal
			Vec teammateToGoal = (ownGoal - teammatePos).Normalized();
			Vec perpendicular = Vec(-teammateToGoal.y, teammateToGoal.x, 0.f).Normalized();  // Fixed: explicit float

			// Fixed: manual vector arithmetic
			Vec goalOffset = Vec(teammateToGoal.x * 800.f, teammateToGoal.y * 800.f, teammateToGoal.z * 800.f);
			Vec perpOffset = Vec(perpendicular.x * 600.f, perpendicular.y * 600.f, perpendicular.z * 600.f);
			Vec supportPos = teammatePos + goalOffset + perpOffset;

			return supportPos;
		}

		float CalculateOpponentAwareness(const Player& player, const TeamAnalysis& analysis, const GameState& state) {
			// Reward positioning that maintains good sight lines to opponents
			Vec playerToOpponentCenter = (analysis.opponentCenterOfMass - player.pos).Normalized();
			Vec playerToBall = (state.ball.pos - player.pos).Normalized();

			float awarenessAngle = playerToOpponentCenter.Dot(playerToBall);

			// Good awareness when you can see both ball and opponents
			return RS_CLAMP(awarenessAngle * 0.5f + 0.5f, 0.f, 1.f);
		}

		float CalculateDynamicCampingPenalty(const Player& player, const Vec& ownGoal, const GameState& state) {
			float distToGoal = (player.pos - ownGoal).Length();

			// Dynamic threshold based on game state
			float minDistFromGoal = 800.f; // Base minimum

			// Adjust based on ball position
			float ballDistFromGoal = (state.ball.pos - ownGoal).Length();
			if (ballDistFromGoal < 2000.f) {
				minDistFromGoal *= 0.7f; // Allow closer positioning when ball is near
			}

			if (distToGoal < minDistFromGoal) {
				float penalty = -0.4f * (1.f - (distToGoal / minDistFromGoal));
				return penalty;
			}

			return 0.f;
		}
	};




	class AirDribbleReward : public Reward {
	public:
		float minHeight;
		float maxHeight;
		float distanceThreshold;

		AirDribbleReward(float minHeight = 350.0f, float maxHeight = 1500.0f, float distanceThreshold = 400.0f)
			: minHeight(minHeight), maxHeight(maxHeight), distanceThreshold(distanceThreshold) {
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (player.isOnGround)
				return 0.0f;

			if (state.ball.pos.z < minHeight)
				return 0.0f;

			float dist = (state.ball.pos - player.pos).Length();
			if (dist > distanceThreshold)
				return 0.0f;

			bool targetOrangeGoal = player.team == Team::BLUE;
			Vec targetPos = targetOrangeGoal ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;
			Vec toGoal = (targetPos - state.ball.pos).Normalized();

			float velProj = state.ball.vel.Dot(toGoal);

			if (velProj <= 0.0f)
				return 0.0f;

			float velReward = velProj / CommonValues::BALL_MAX_SPEED;

			float clampedHeight = std::min(state.ball.pos.z, maxHeight);
			float heightScale = clampedHeight / maxHeight;

			return velReward * heightScale;
		}
	};

	class KickoffSpeedflipReward : public Reward {
	private:
		struct PlayerState {
			bool kickoffDetected = false;
			bool jumpUsed = false;
			float jumpTime = -1.0f;
			bool diagonalInputDetected = false;
			bool flipCancelDetected = false;
			bool speedflipCompleted = false;
			Vec lastPos = Vec(0, 0, 0);
			float lastTime = 0.0f;
			Vec velocityBeforeFlip = Vec(0, 0, 0);
			Vec velocityAfterFlip = Vec(0, 0, 0);
		};
		std::map<uint32_t, PlayerState> playerStates;
		float kickoffStartTime = -1.0f;
		int frameCount = 0;

	public:
		float kickoffDetectionTime;
		float maxJumpDelay;
		float minSpeedThreshold;
		float diagonalInputThreshold;
		float cancelThreshold;
		float maxCancelDelay;
		float rewardValue;
		bool debug;

		KickoffSpeedflipReward(
			float kickoffDetectionTime = 3.0f,
			float maxJumpDelay = 0.5f,
			float minSpeedThreshold = 1200.0f,
			float diagonalInputThreshold = 0.3f,
			float cancelThreshold = -0.5f,
			float maxCancelDelay = 0.3f,
			float rewardValue = 5.0f,
			bool debug = false
		) : kickoffDetectionTime(kickoffDetectionTime), maxJumpDelay(maxJumpDelay),
			minSpeedThreshold(minSpeedThreshold), diagonalInputThreshold(diagonalInputThreshold),
			cancelThreshold(cancelThreshold), maxCancelDelay(maxCancelDelay),
			rewardValue(rewardValue), debug(debug) {
		}

		virtual void Reset(const GameState& initialState) override {
			playerStates.clear();
			kickoffStartTime = -1.0f;
			frameCount = 0;
			for (const auto& player : initialState.players) {
				PlayerState& state = playerStates[player.carId];
				state.kickoffDetected = false;
				state.jumpUsed = false;
				state.jumpTime = -1.0f;
				state.diagonalInputDetected = false;
				state.flipCancelDetected = false;
				state.speedflipCompleted = false;
				state.lastPos = player.pos;
				state.lastTime = 0.0f;
				state.velocityBeforeFlip = Vec(0, 0, 0);
				state.velocityAfterFlip = Vec(0, 0, 0);
			}
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			uint32_t carId = player.carId;

			if (playerStates.find(carId) == playerStates.end()) {
				PlayerState& newState = playerStates[carId];
				newState.kickoffDetected = false;
				newState.jumpUsed = false;
				newState.jumpTime = -1.0f;
				newState.diagonalInputDetected = false;
				newState.flipCancelDetected = false;
				newState.speedflipCompleted = false;
				newState.lastPos = player.pos;
				newState.lastTime = 0.0f;
				newState.velocityBeforeFlip = Vec(0, 0, 0);
				newState.velocityAfterFlip = Vec(0, 0, 0);
			}

			PlayerState& st = playerStates[carId];
			float reward = 0.0f;
			float currentTime = frameCount * 0.016f;
			float dt = (st.lastTime > 0) ? (currentTime - st.lastTime) : 0.016f;
			frameCount++;

			// DÃ©tecter le kickoff
			if (!st.kickoffDetected && IsKickoffPosition(player.pos)) {
				st.kickoffDetected = true;
				kickoffStartTime = currentTime;

			}

			// DÃ©tecter le saut (avec condition kickoff)
			if (st.kickoffDetected && !st.jumpUsed && player.HasFlipOrJump()) {
				float jumpDelay = (kickoffStartTime >= 0) ? (currentTime - kickoffStartTime) : 0;
				if (jumpDelay <= maxJumpDelay) {
					st.jumpUsed = true;
					st.jumpTime = currentTime;
					st.velocityBeforeFlip = player.vel; // Sauvegarder la vÃ©locitÃ© avant le flip

				}
			}

			// DÃ©tecter l'input diagonal aprÃ¨s le saut (via la vÃ©locitÃ©)
			if (st.jumpUsed && !st.diagonalInputDetected) {
				// VÃ©rifier si la voiture se dÃ©place en diagonal (composantes X et Y significatives)
				float velX = fabsf(player.vel.x);
				float velY = fabsf(player.vel.y);
				float velZ = fabsf(player.vel.z);

				// Le speedflip au kickoff a des composantes X et Y Ã©levÃ©es avec peu de Z
				if (velX > diagonalInputThreshold * 1000.0f && velY > diagonalInputThreshold * 1000.0f && velZ < 500.0f) {
					st.diagonalInputDetected = true;

				}
			}

			// DÃ©tecter le flip cancel (via la vÃ©locitÃ© et la rotation)
			if (st.jumpUsed && st.diagonalInputDetected && !st.flipCancelDetected) {
				float cancelDelay = (st.jumpTime >= 0) ? (currentTime - st.jumpTime) : 0;
				if (cancelDelay <= maxCancelDelay) {
					// Le flip cancel se caractÃ©rise par une vÃ©locitÃ© Z qui reste basse
					// et une rotation de la voiture qui s'arrÃªte
					if (fabsf(player.vel.z) < 300.0f && player.rotMat.up.z > 0.7f) {
						st.flipCancelDetected = true;
						st.velocityAfterFlip = player.vel; // Sauvegarder la vÃ©locitÃ© aprÃ¨s le flip

					}
				}
			}

			// VÃ©rifier le speedflip complet (accÃ©lÃ©ration et direction)
			if (st.jumpUsed && st.diagonalInputDetected && st.flipCancelDetected && !st.speedflipCompleted) {
				// Calculer l'accÃ©lÃ©ration due au speedflip
				float speedBefore = st.velocityBeforeFlip.Length();
				float speedAfter = st.velocityAfterFlip.Length();
				float acceleration = (speedAfter - speedBefore) / dt;

				// VÃ©rifier que la vitesse est suffisante et qu'il y a eu accÃ©lÃ©ration
				if (speedAfter >= minSpeedThreshold && acceleration > 500.0f) {
					st.speedflipCompleted = true;
					reward = rewardValue;

				}
			}

			st.lastPos = player.pos;
			st.lastTime = currentTime;

			return reward;
		}

	private:
		bool IsKickoffPosition(const Vec& playerPos) {
			// Positions de kickoff exactes
			Vec kickoffPositions[] = {
				Vec(0, -5120, 0),      // Centre
				Vec(-2048, -5120, 0),  // Gauche
				Vec(2048, -5120, 0),   // Droite
				Vec(-2048, 5120, 0),   // Gauche adverse
				Vec(2048, 5120, 0)     // Droite adverse
			};

			for (const auto& pos : kickoffPositions) {
				if ((playerPos - pos).Length() < 500.0f) {
					return true;
				}
			}
			return false;
		}

		float CalculateSpeed(const Vec& currentPos, const Vec& lastPos, float dt) {
			if (dt <= 0) return 0.0f;
			Vec displacement = currentPos - lastPos;
			return displacement.Length() / dt;
		}
	};


	class GoodGoalPlacementReward : public Reward { //pretty good
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			Vec goalPos = (player.team == Team::BLUE) ? CommonValues::ORANGE_GOAL_CENTER : CommonValues::BLUE_GOAL_CENTER;

			float closestDist = FLT_MAX;
			Vec closestEnemyPos = goalPos;

			for (const auto& other : state.players) {
				if (other.team != player.team) {
					float dist = other.pos.Dist(goalPos);
					if (dist < closestDist) {
						closestDist = dist;
						closestEnemyPos = other.pos;
					}
				}
			}

			std::vector<float> xs = linspace(-801.505f, 801.505f, 10);
			std::vector<float> zs = linspace(91.25f, 551.525f, 10);
			float y = goalPos.y;

			float maxMinDist = -1.0f;
			Vec bestPoint = { 0, y, 0 };

			for (float x : xs) {
				for (float z : zs) {
					float minDistToAnyEnemy = FLT_MAX;
					for (const auto& other : state.players) {
						if (other.team != player.team) {
							float dist = distance2D(x, z, other.pos.x, other.pos.z);
							minDistToAnyEnemy = std::min(minDistToAnyEnemy, dist);
						}
					}
					if (minDistToAnyEnemy > maxMinDist) {
						maxMinDist = minDistToAnyEnemy;
						bestPoint = { x, y, z };
					}
				}
			}

			Vec bPos = state.ball.pos;

			float reward = 0.0f;

			if (state.goalScored && state.ball.pos.Dist(goalPos) < 2000) {
				float dstToTarget = (bPos - bestPoint).Length();
				reward = std::max(0.0f, 1.0f - (dstToTarget / 1200.0f));

				if (reward > 0.0f) {
					float velScale = std::clamp(state.ball.vel.Length() / CommonValues::BALL_MAX_SPEED, 0.0f, 1.0f);
					reward += velScale;
					reward = std::clamp(reward, 0.0f, 2.0f);
				}
			}
			return reward;
		}
	};

	class JumpTouchReward : public Reward {
	public:
		float minHeight, maxHeight, range;

		JumpTouchReward(float minHeight = 150.75f, float maxHeight = 300.0f)
			: minHeight(minHeight), maxHeight(maxHeight) {
			range = maxHeight - minHeight;
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (player.ballTouchedStep && !player.isOnGround && state.ball.pos.z >= minHeight) {
				return (state.ball.pos.z - minHeight) / range;
			}
			return 0;
		}
	};

	inline Vec FindBestGoalPoint(const Player& player, const GameState& state)
	{
		const Vec goalPos = (player.team == Team::BLUE)
			? CommonValues::ORANGE_GOAL_CENTER
			: CommonValues::BLUE_GOAL_CENTER;

		auto isOpponent = [&](const Player& me, const Player& other) {
			return me.team != other.team;
			};

		// search grid inside the goal
		const std::vector<float> xs = linspace(-801.505f, 801.505f, 10);
		const std::vector<float> zs = linspace(91.25f, 551.525f, 10);
		const float y = goalPos.y;

		float bestMinDst = -1.f;
		Vec   bestPoint = { 0, y, 0 };

		for (float x : xs)
			for (float z : zs) {
				float minToEnemy = FLT_MAX;
				for (const auto& other : state.players)
					if (isOpponent(player, other))
						minToEnemy = std::min(minToEnemy,
							distance2D(x, z, other.pos.x, other.pos.z));

				if (minToEnemy > bestMinDst) {
					bestMinDst = minToEnemy;
					bestPoint = { x, y, z };
				}
			}
		return bestPoint;
	}


}