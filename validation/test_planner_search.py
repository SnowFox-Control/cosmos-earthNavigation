"""Budget/seed invariants and independent CPU/CUDA fixture consistency."""
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.io import loadmat

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))
from compare_planners import search, reflect


class PlannerSearchTests(unittest.TestCase):
    def test_equal_budget_and_seed_reproducibility(self):
        initial=np.random.default_rng(8).random((12,10))
        for method in ['PSO','GA','SA']:
            counter=[0]
            def objective(x):
                counter[0]+=len(x)
                self.assertTrue(np.all((x>=0)&(x<=1)))
                return np.sum((x-.3)**2,axis=1)
            a=search(method,initial,objective,161,4)
            self.assertEqual(counter[0],60)
            b=search(method,initial,objective,161,4)
            self.assertEqual(a['x'],b['x'])
            self.assertEqual(a['history'],b['history'])
            self.assertTrue(np.all(np.diff(np.array(a['history'])[:,2])<=0))
    def test_reflection_handles_multiple_crossings(self):
        self.assertTrue(np.allclose(reflect(np.array([-2.2,-.2,1.2,2.2])),[.2,.2,.8,.2]))
    def test_cached_cpu_cuda_agreement(self):
        cpu=loadmat(ROOT/'results/comparison/numba_probe.mat')
        gpu=loadmat(ROOT/'results/comparison/cuda_probe.mat')
        self.assertTrue(np.allclose(cpu['scores'],gpu['scores'],rtol=1e-8,atol=1e-8))
        self.assertTrue(np.allclose(cpu['terminal'],gpu['terminal'],rtol=1e-10,atol=1e-7,equal_nan=True))
    def test_archived_protocol(self):
        import json
        campaign=json.loads((ROOT/'results/comparison/campaign.json').read_text())
        self.assertFalse(campaign['localRefinement'])
        self.assertEqual(len(campaign['entries']),15)
        for seed in range(161,166):
            entries=[x for x in campaign['entries'] if x['seed']==seed]
            self.assertEqual(len(entries),3)
            self.assertEqual({x['evaluations'] for x in entries},{17376})
            for x in entries[1:]:
                self.assertEqual(x['initial'],entries[0]['initial'])
        verified=json.loads((ROOT/'results/comparison/verification.json').read_text())['entries']
        self.assertEqual(sum(x['success'] for x in verified if x['method']=='PSO'),5)


if __name__=='__main__':unittest.main()
