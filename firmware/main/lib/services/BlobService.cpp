// BlobService.cpp — binary blob detection on QQVGA grayscale frames.
#include "BlobService.h"
#include <esp_heap_caps.h>
#include <string.h>
#include <math.h>

// ── union-find (path-compressed, union-by-rank) ───────────────────────────────

static int uf_find(int* parent, int x) {
    while (parent[x] != x) {
        parent[x] = parent[parent[x]]; // path halving
        x = parent[x];
    }
    return x;
}

static void uf_union(int* parent, int* rank, int a, int b) {
    a = uf_find(parent, a);
    b = uf_find(parent, b);
    if (a == b) return;
    if (rank[a] < rank[b]) { int t = a; a = b; b = t; }
    parent[b] = a;
    if (rank[a] == rank[b]) rank[a]++;
}

// ── Otsu threshold ────────────────────────────────────────────────────────────

uint8_t BlobService::otsu_(const uint8_t* img, int n) {
    uint32_t hist[256] = {};
    for (int i = 0; i < n; i++) hist[img[i]]++;

    uint64_t total = 0;
    for (int i = 0; i < 256; i++) total += (uint64_t)i * hist[i];

    uint64_t sumB = 0;
    uint32_t wB = 0, wF = 0;
    float maxVar = 0.0f;
    uint8_t best = 128;

    for (int t = 0; t < 256; t++) {
        wB += hist[t];
        if (!wB) continue;
        wF = (uint32_t)n - wB;
        if (!wF) break;
        sumB += (uint64_t)t * hist[t];
        const float mB = (float)sumB / wB;
        const float mF = (float)(total - sumB) / wF;
        const float diff = mB - mF;
        const float var  = (float)wB * (float)wF * diff * diff;
        if (var > maxVar) { maxVar = var; best = (uint8_t)t; }
    }
    return best;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

bool BlobService::begin() { return true; }

void BlobService::onActivate() {
    const int N = kW * kH;
    binary_ = (uint8_t*)heap_caps_malloc(N,              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    label_  = (int*)    heap_caps_malloc(N * sizeof(int), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    parent_ = (int*)    heap_caps_malloc(N * sizeof(int), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    rank_   = (int*)    heap_caps_malloc(N * sizeof(int), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!binary_ || !label_ || !parent_ || !rank_) {
        Serial.println("[Blob] PSRAM alloc failed");
    } else {
        Serial.println("[Blob] ready");
    }
}

void BlobService::onDeactivate() {
    heap_caps_free(binary_); binary_ = nullptr;
    heap_caps_free(label_);  label_  = nullptr;
    heap_caps_free(parent_); parent_ = nullptr;
    heap_caps_free(rank_);   rank_   = nullptr;
}

// ── per-frame detection ───────────────────────────────────────────────────────

void BlobService::onFrame(camera_fb_t* fb) {
    if (!binary_ || !label_ || !parent_ || !rank_) return;

    const uint8_t* src = fb->buf;
    const int N = kW * kH;

    // 1. Threshold
    const uint8_t thr = autoThr_ ? otsu_(src, N) : fixedThr_;
    for (int i = 0; i < N; i++) binary_[i] = (src[i] > thr) ? 1 : 0;

    // 2. Two-pass connected components (4-connected)
    for (int i = 0; i < N; i++) { parent_[i] = i; rank_[i] = 0; }

    for (int y = 0; y < kH; y++) {
        for (int x = 0; x < kW; x++) {
            const int idx = y * kW + x;
            if (!binary_[idx]) { label_[idx] = -1; continue; }
            label_[idx] = idx;
            if (x > 0   && binary_[idx - 1])    uf_union(parent_, rank_, idx, idx - 1);
            if (y > 0   && binary_[idx - kW])   uf_union(parent_, rank_, idx, idx - kW);
        }
    }

    // 3. Collect blob stats (area, bounding box, centroid sum)
    struct Stats {
        int sum_x, sum_y, area;
        int x0, y0, x1, y1;
        bool used;
    };

    // Use a flat array indexed by root label (only some roots will be used).
    // Since root values are in [0, N), we track only up to kMaxBlobs unique ones.
    static constexpr int kMaxRoots = BlobService::kMaxBlobs * 4;
    int   roots[kMaxRoots] = {};
    Stats stats[kMaxRoots] = {};
    int   nRoots = 0;

    auto findSlot = [&](int root) -> int {
        for (int i = 0; i < nRoots; i++) if (roots[i] == root) return i;
        if (nRoots >= kMaxRoots) return -1;
        roots[nRoots] = root;
        stats[nRoots] = { 0, 0, 0, kW, kH, -1, -1, true };
        return nRoots++;
    };

    for (int y = 0; y < kH; y++) {
        for (int x = 0; x < kW; x++) {
            const int idx = y * kW + x;
            if (label_[idx] < 0) continue;
            const int root = uf_find(parent_, idx);
            const int slot = findSlot(root);
            if (slot < 0) continue;
            Stats& s = stats[slot];
            s.sum_x += x; s.sum_y += y; s.area++;
            if (x < s.x0) s.x0 = x;
            if (x > s.x1) s.x1 = x;
            if (y < s.y0) s.y0 = y;
            if (y > s.y1) s.y1 = y;
        }
    }

    // 4. Build result array, filter tiny blobs (< 9 px), sort by area desc
    Blob blobs[kMaxBlobs];
    int  nBlobs = 0;

    for (int i = 0; i < nRoots && nBlobs < kMaxBlobs; i++) {
        const Stats& s = stats[i];
        if (s.area < 9) continue;
        Blob& b  = blobs[nBlobs++];
        b.cx     = s.sum_x / s.area;
        b.cy     = s.sum_y / s.area;
        b.area   = s.area;
        b.x0     = s.x0; b.y0 = s.y0;
        b.x1     = s.x1; b.y1 = s.y1;
        const int w = s.x1 - s.x0 + 1;
        const int h = s.y1 - s.y0 + 1;
        b.aspect = (h > 0) ? (float)w / h : 1.0f;
    }

    // Insertion sort (small array, fine here)
    for (int i = 1; i < nBlobs; i++) {
        Blob tmp = blobs[i];
        int j = i - 1;
        while (j >= 0 && blobs[j].area < tmp.area) { blobs[j + 1] = blobs[j]; j--; }
        blobs[j + 1] = tmp;
    }

    if (resultCb_) resultCb_(blobs, nBlobs);
}
