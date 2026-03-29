// import logo from './logo.svg';
import './App.css';
import React, { useState } from 'react';
import { motion } from 'framer-motion';
import LoaderPage from "./LoaderPage";

export default function App() {
  const [started, setStarted] = useState(false);

  if (!started) {
    return (
      <div className="min-h-screen bg-gradient-to-br from-gray-50 to-gray-100 flex flex-col items-center justify-center px-6 py-10">
        <header className="w-full max-w-5xl flex justify-between items-center mb-12">
          <h1 className="text-2xl font-semibold text-gray-800">APEX Labs</h1>
          <span className="text-sm text-orange-600 font-medium">SYSTEM READY</span>
        </header>

        <div className="w-full max-w-5xl shadow-xl border border-gray-200 rounded-2xl bg-white p-10">
          <div className="mb-8">
            <h2 className="text-3xl font-bold text-gray-800 flex items-center gap-2">
              Welcome to Apex
              <span className="bg-orange-100 text-orange-600 text-sm font-semibold px-2 py-0.5 rounded-md">Alpha</span>
            </h2>
            <p className="text-gray-600 mt-2 max-w-3xl">
              AI-powered laboratory automation platform. Apex combines advanced AI planning, computer vision, and precision robotics
              to automate laboratory workflows — from pipetting to automated plate handling.
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 mb-10">
            {[
              {
                title: 'Vision Understanding',
                desc: 'Precise object detection and spatial analysis with high-fidelity environment mapping.',
                icon: '👁️',
              },
              {
                title: 'Intelligent Planning',
                desc: 'Advanced LLM planners generate optimized laboratory protocols with contextual awareness.',
                icon: '🧠',
              },
              {
                title: 'Robotic Execution',
                desc: 'Precision control systems with real-time feedback for reliable laboratory workflow automation.',
                icon: '🤖',
              },
            ].map((item, i) => (
              <motion.div
                key={i}
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: i * 0.1 }}
                className="p-5 border border-gray-200 rounded-xl hover:shadow-md transition bg-gray-50/60"
              >
                <div className="text-3xl mb-3">{item.icon}</div>
                <h3 className="text-lg font-semibold text-gray-800 mb-1">{item.title}</h3>
                <p className="text-gray-600 text-sm leading-relaxed">{item.desc}</p>
              </motion.div>
            ))}
          </div>

          <div className="flex justify-between items-center">
            <p className="text-gray-600 text-sm">Getting Started — Begin by mapping your workspace objects.</p>
            <button
              onClick={() => setStarted(true)}
              className="bg-orange-600 hover:bg-orange-700 text-white px-6 py-2 rounded-lg"
            >
              Begin →
            </button>
          </div>
        </div>

        <div className="mt-6 text-sm text-green-600 font-medium">AI system initialized</div>
      </div>
    );
  }

  if (started === 'loader') {
    return <LoaderPage />;
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-gray-50 to-gray-100 flex flex-col items-center px-6 py-10">
      <header className="w-full max-w-4xl flex justify-between items-center mb-8">
        <h1 className="text-2xl font-semibold text-gray-800">APEX Labs</h1>
        <span className="text-sm text-green-600 font-medium">AI Planning Ready</span>
      </header>

      <div className="w-full max-w-4xl bg-white shadow-lg border border-gray-200 rounded-2xl p-8">
        <div className="bg-red-50 border border-red-200 text-red-700 px-4 py-3 rounded-lg mb-6">
          <strong>Objects detected:</strong> 3 items with spatial coordinates loaded.
        </div>

        <div className="mb-6">
          <label className="block text-gray-700 font-medium mb-2">Experiment Description</label>
          <textarea
            placeholder="For each of my 3 samples, carry out a 10x serial dilution from..."
            rows="4"
            className="w-full border-2 border-orange-300 focus:border-orange-500 rounded-lg p-3 outline-none text-gray-800"
          ></textarea>
          <p className="text-sm text-gray-500 mt-1">
            Be specific about what substances to manipulate and what procedures to perform.
          </p>
        </div>

        <div className="flex justify-between items-center">
          <button
            onClick={() => setStarted(false)}
            className="text-gray-600 hover:text-gray-800 text-sm"
          >
            ← Back
          </button>
          <button
            onClick={() => setStarted('loader')}
            className="bg-orange-600 hover:bg-orange-700 text-white px-6 py-2 rounded-lg"
          >
            Continue →
          </button>
        </div>
      </div>

      <p className="text-gray-500 text-sm mt-6 italic">*No coding required*</p>
    </div>
  );
}
