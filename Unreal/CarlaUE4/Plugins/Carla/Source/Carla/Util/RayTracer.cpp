// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Util/RayTracer.h"

#include "Carla/Game/CarlaStatics.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "DrawDebugHelpers.h"


namespace crp = carla::rpc;

std::vector<crp::LabelledPoint> URayTracer::CastRay(
    FVector StartLocation, FVector EndLocation, UWorld * World)
{
  TArray<FHitResult> OutHits;
  World->LineTraceMultiByChannel(
      OutHits,
      StartLocation,
      EndLocation,
      ECC_GameTraceChannel3, // overlap channel
      FCollisionQueryParams(),
      FCollisionResponseParams()
  );
  std::vector<crp::LabelledPoint> result;
  for (auto& Hit : OutHits)
  {
    UPrimitiveComponent* Component = Hit.GetComponent();
    crp::CityObjectLabel ComponentTag =
        ATagger::GetTagOfTaggedComponent(*Component);
    result.emplace_back(crp::LabelledPoint(Hit.Location, ComponentTag));
  }
  return result;
}

std::pair<bool, crp::LabelledPoint> URayTracer::ProjectPoint(
    FVector StartLocation, FVector Direction, float MaxDistance, UWorld * World)
{
  FHitResult Hit;
  bool bDidHit = World->LineTraceSingleByChannel(
      Hit,
      StartLocation,
      StartLocation + Direction.GetSafeNormal() * MaxDistance,
      ECC_GameTraceChannel2, // camera
      FCollisionQueryParams(),
      FCollisionResponseParams()
  );
  if (bDidHit)
  {
    UPrimitiveComponent* Component = Hit.GetComponent();
    crp::CityObjectLabel ComponentTag =
        ATagger::GetTagOfTaggedComponent(*Component);
    return std::make_pair(bDidHit, crp::LabelledPoint(Hit.Location, ComponentTag));
  }
  return std::make_pair(bDidHit, crp::LabelledPoint(FVector(0.0f,0.0f,0.0f), crp::CityObjectLabel::None));


  }

std::vector<crp::LabelledPoint> URayTracer::ProjectPoints(
    const std::vector<FVector>& StartLocations,
    FVector Direction,
    float MaxDistance,
    UWorld * World,
    const std::vector<const AActor *>& IgnoredActors)
{
    std::vector<crp::LabelledPoint> Result;
    Result.reserve(StartLocations.size());

    // prepare for the ray tracing
    FHitResult Hit;

    FCollisionQueryParams QueryParams;
    QueryParams.bReturnPhysicalMaterial = true;
    for (const AActor * actor : IgnoredActors) {
        QueryParams.AddIgnoredActor(actor);
    }

    for (const auto & StartLocation : StartLocations) {
        bool bDidHit = World->LineTraceSingleByChannel(
            Hit,
            StartLocation,
            StartLocation + Direction.GetSafeNormal() * MaxDistance,
            ECC_Visibility,
            QueryParams
        );

        if (!bDidHit)
        {
            Result.emplace_back(FVector(0.0f, 0.0f, 0.0f), crp::CityObjectLabel::None);
            continue;
        }

        UPrimitiveComponent * Component = Hit.GetComponent();
        crp::CityObjectLabel ComponentTag = ATagger::GetTagOfTaggedComponent(*Component);

        UPhysicalMaterial * Material = Hit.PhysMaterial.Get();
        float Friction = Material ? Material->Friction : -1.0f;

        Result.emplace_back(Hit.Location, ComponentTag, crp::Vector3D(Hit.Normal.X, Hit.Normal.Y, Hit.Normal.Z), Friction);
    }

    return Result;
}
