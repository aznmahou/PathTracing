/*
	This file is part of TinyRender, an educative rendering system.

	Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
	Derek Nowrouzezahrai, McGill University.
*/

#pragma once

TR_NAMESPACE_BEGIN

/**
 * Path tracer integrator
 */
	struct PathTracerIntegrator : Integrator {
	explicit PathTracerIntegrator(const Scene& scene) : Integrator(scene) {
		m_isExplicit = scene.config.integratorSettings.pt.isExplicit;
		m_maxDepth = scene.config.integratorSettings.pt.maxDepth;
		m_rrDepth = scene.config.integratorSettings.pt.rrDepth;
		m_rrProb = scene.config.integratorSettings.pt.rrProb;


	}

	static inline float balanceHeuristic(float nf, float fPdf, float ng, float gPdf) {
		float f = nf * fPdf, g = ng * gPdf;
		return f / (f + g);
	}

	v3f renderImplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
		v3f Li(0.f);
		SurfaceInteraction intersection = hit;
		v3f throughput(1.0f);
		v2f sample = sampler.next2D();

		float emPdf;
		size_t id = selectEmitter(sampler.next(), emPdf);
		const Emitter& em = getEmitterByID(id);
		
		//if we hit a light just return light value
		v3f emissionHit = getEmission(intersection);
		if (emissionHit != v3f(0.0f)) {
			Li += emissionHit / emPdf;
		}
		//if we dont hit light then we do our first bounce
		else {
			bool lightHit = false;
			v3f throughput(1.f);
			for (int i = 0; i < m_maxDepth; i++) {

				v2f sample = sampler.next2D();
				float emPdf;
				size_t id = selectEmitter(sampler.next(), emPdf);
				const Emitter& em = getEmitterByID(id);

				const BSDF* bsdf = getBSDF(intersection);
				float pdf;
				float* pdfPointer = &pdf;
				v3f fr = bsdf->sample(intersection, sample, pdfPointer);
				v3f wi = intersection.wi;
				wi = intersection.frameNs.toWorld(wi);
				intersection.wi = wi;

				Ray sampleRay = Ray(intersection.p, wi);
				scene.bvh->intersect(sampleRay, intersection);

				//if our bounce hits light return light
				emissionHit = getEmission(intersection);
				if (emissionHit != v3f(0.0f)) {
					Li += emissionHit / emPdf *throughput * fr;
					i = m_maxDepth; //if we hit a light we  get our stuff  set the loop to max so we end
					lightHit = true;
				}
				else {
					throughput *= fr;
				}

			}
			if (lightHit == false) {
				return v3f(0.f);
			}
		}
	
		return Li;
	}

	v3f renderExplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
		v3f Li(0.f);

		SurfaceInteraction inter = hit;
		v3f throughput(1.0f);
		v2f sample = sampler.next2D();

		float emPdf;
		size_t id = selectEmitter(sampler.next(), emPdf);
		const Emitter& em = getEmitterByID(id);

		
		v3f emissionHit = getEmission(inter);
		if (emissionHit != v3f(0.0f)) {
			Li += emissionHit / emPdf;
		}
		else {

			//do the looping 

			SurfaceInteraction directInter = inter;

			for (int bounces = 0 ;; bounces++) { //removed  bounces < m_maxDepth  , need to add manual break condition
				
				if (m_maxDepth != -1) { // if m_maxDepth is not -1 then we check if we exceed maxDepth if it is -1 we dont check and the loop ending depends on russian roulette
					if (bounces >= m_maxDepth) {
						break;
					}
				}

				
				v2f sample = sampler.next2D();

				float emPdf;
				size_t id = selectEmitter(sampler.next(), emPdf);
				const Emitter& em = getEmitterByID(id);

				//Direct Lighting

				v3f position;
				v3f normal;
				float pdf;
				sampleEmitterPosition(sampler, em, normal, position, pdf);;


				v3f sampleDir = glm::normalize(position - directInter.p);
				v3f samplingPos = directInter.p;
				sampleDir = glm::normalize(sampleDir);

				directInter.wi = directInter.frameNs.toLocal((sampleDir));

				Ray sampleRay = Ray(directInter.p, sampleDir);

				const BSDF* bsdf = getBSDF(directInter);

				v3f fr = bsdf->eval(directInter);


				bool shadHit = scene.bvh->intersect(sampleRay, directInter);

				//if we hit light we add light
				v3f emissionVal = getEmission(directInter);
				if (emissionVal != v3f(0.0f)) {

					float dotProd = glm::max(0.0f,glm::dot(-sampleDir, normal));
					float jacobian = dotProd / glm::length2(samplingPos - position);
					Li += (emissionVal / emPdf) *throughput *fr / pdf * jacobian;
				}

				//Indirect Lighting



				const BSDF* bsdfIndir = getBSDF(inter);
				float pdfBSDF;
				float* pdfPointer = &pdfBSDF;
				v3f frBSDF = bsdfIndir->sample(inter, sample, pdfPointer);
				v3f wi = inter.wi;
				wi = inter.frameNs.toWorld(wi);
				inter.wi = wi;
				throughput *= frBSDF;

				//shoot new ray
				Ray sampleRayBSDF = Ray(inter.p, wi);
				scene.bvh->intersect(sampleRayBSDF, inter);

				emissionVal = getEmission(inter);
				if (emissionVal != v3f(0.0f)) {
					break;
				}

				//update the directInter to be always where the sampleRayBSDF hits
				directInter = inter;

				//we need to add some stuff if the bsdf ray hits a light just break out cuz we done
				
				if (bounces > m_rrDepth) {//we start rr at depth = 4
					if (sampler.next() > m_rrProb) {
						break;
					}
					throughput *= 1 / m_rrProb;
				}
				//if we recurse 2 much just break remove after this test
				//if (bounces > 15) {
				//	break;
				//}

			}

		}
		

		//returning 

		return Li;
	}

	/*
	v3f renderExplicitMIS(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
		v3f Li(0.f);


		SurfaceInteraction inter = hit;
		v3f throughput(1.0f);
		v2f sample = sampler.next2D();

		float emPdf;
		size_t id = selectEmitter(sampler.next(), emPdf);
		const Emitter& em = getEmitterByID(id);


		v3f emissionHit = getEmission(inter);
		if (emissionHit != v3f(0.0f)) {
			Li += emissionHit / emPdf;
		}
		else {

			//do the looping 

			SurfaceInteraction directInter = inter;

			for (int bounces = 0;; bounces++) { //removed  bounces < m_maxDepth  , need to add manual break condition

				if (m_maxDepth != -1) { // if m_maxDepth is not -1 then we check if we exceed maxDepth if it is -1 we dont check and the loop ending depends on russian roulette
					if (bounces >= m_maxDepth) {
						break;
					}
				}


				v2f sample = sampler.next2D();

				float emPdf;
				size_t id = selectEmitter(sampler.next(), emPdf);
				const Emitter& em = getEmitterByID(id);

				v3f Lemm(0.f);
				v3f Lbdsf(0.f);
				float nf = m_emitterSamples;
				float ng = m_bsdfSamples;

				SurfaceInteraction emitterInter = directInter;
				SurfaceInteraction bsdfInter = directInter;
				//Direct Lighting
				//Emitter Sampling
				for (size_t i = 0; i < m_emitterSamples; i++) {

					v3f position;
					v3f normal;
					float fpdf;
					sampleEmitterPosition(sampler, em, normal, position, fpdf);;


					v3f sampleDir = glm::normalize(position - emitterInter.p);
					v3f samplingPos = emitterInter.p;
					sampleDir = glm::normalize(sampleDir);

					emitterInter.wi = emitterInter.frameNs.toLocal((sampleDir));

					Ray sampleRay = Ray(emitterInter.p, sampleDir);

					const BSDF* bsdf = getBSDF(emitterInter);

					v3f fr = bsdf->eval(emitterInter);
					float gpdf = bsdf->pdf(emitterInter);

					bool shadHit = scene.bvh->intersect(sampleRay, emitterInter);

					//if we hit light we add light
					v3f emissionVal = getEmission(emitterInter);
					if (emissionVal != v3f(0.0f)) {

						float weight = balanceHeuristic(nf, fpdf, ng, gpdf);

						float dotProd = glm::max(0.0f, glm::dot(-sampleDir, normal));
						float jacobian = dotProd / glm::length2(samplingPos - position);
						Lemm += (emissionVal / emPdf) *throughput *fr / fpdf * jacobian * weight;
					}


				}
				if (m_emitterSamples != 0) {
					Li += Lemm / (float)m_emitterSamples;

				}
				//BSDF Sampling
				for (size_t j = 0; j < m_bsdfSamples; j++) {

					v2f sample = sampler.next2D();

					v3f position;
					v3f normal;
					float fpdf;
					sampleEmitterPosition(sampler, em, normal, position, fpdf);;

					const BSDF* bsdf = getBSDF(bsdfInter);
					float gPdf;
					v3f fr = bsdf->sample(bsdfInter, sample, &gPdf);
					v3f wi = bsdfInter.wi;
					wi = bsdfInter.frameNs.toWorld(wi);


					Ray sampleRay = Ray(bsdfInter.p, wi);


					bool shadHit = scene.bvh->intersect(sampleRay, bsdfInter);

					v3f emissionVal = getEmission(bsdfInter);
					if (emissionVal != v3f(0.0f)) {

						float weight = balanceHeuristic(ng, gPdf, nf, fpdf);
						Lbdsf += (emissionVal / emPdf) *throughput *fr * weight;

						
					}
				}
			
				if (m_bsdfSamples != 0) {
					Li += Lbdsf / (float)m_bsdfSamples;
				}


				//Indirect Lighting



				const BSDF* bsdfIndir = getBSDF(inter);
				float pdfBSDF;
				float* pdfPointer = &pdfBSDF;
				v3f frBSDF = bsdfIndir->sample(inter, sample, pdfPointer);
				v3f wi = inter.wi;
				wi = inter.frameNs.toWorld(wi);
				inter.wi = wi;
				throughput *= frBSDF;

				//shoot new ray
				Ray sampleRayBSDF = Ray(inter.p, wi);
				scene.bvh->intersect(sampleRayBSDF, inter);

				v3f emissionVal = getEmission(inter);
				if (emissionVal != v3f(0.0f)) {
					break;
				}

				//update the directInter to be always where the sampleRayBSDF hits
				directInter = inter;

				//we need to add some stuff if the bsdf ray hits a light just break out cuz we done

				if (bounces > m_rrDepth) {//we start rr at depth = 4
					if (sampler.next() > m_rrProb) {
						break;
					}
					throughput *= 1 / m_rrProb;
				}
				//if we recurse 2 much just break remove after this test
				//if (bounces > 15) {
				//	break;
				//}

			}

		}


		//returning 

		return Li;
	}

	*/

	v3f render(const Ray& ray, Sampler& sampler) const override {
		Ray r = ray;
		SurfaceInteraction hit;

		if (scene.bvh->intersect(r, hit)) {
			if (m_isExplicit)
				return this->renderExplicit(ray, sampler, hit);
			else
				return this->renderImplicit(ray, sampler, hit);
		}
		return v3f(0.0);
	}
	int m_maxDepth;     // Maximum number of bounces
	int m_rrDepth;      // When to start Russian roulette
	float m_rrProb;     // Russian roulette probability
	bool m_isExplicit;  // Implicit or explicit


};

TR_NAMESPACE_END
