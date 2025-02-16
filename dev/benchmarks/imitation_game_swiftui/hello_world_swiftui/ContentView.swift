// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import SwiftUI

struct ContentView: View {
    @State private var items: [String] = Array(repeating: "Hello", count: 50)
    @State private var isLoadingMore = false
    private let fetchThreshold = 5

    var body: some View {
        NavigationView { // Add a NavigationView for better view management
            List {
                ForEach(items.indices, id: \.self) { index in
                    Text(items[index])
                        .onAppear {
                            if index == items.count - fetchThreshold {
                                loadMoreContent()
                            }
                        }
                }

                if isLoadingMore {
                    ProgressView()
                        .frame(maxWidth: .infinity) // Center the indicator
                }
            }
            .navigationTitle("Infinite Scroll") // Set a title for clarity
        }
    }

    func loadMoreContent() {
        if isLoadingMore {
            return
        }

        isLoadingMore = true

        let newItems = Array(repeating: "Hello", count: 20) // Fetch 20 more items
        items.append(contentsOf: newItems)
        isLoadingMore = false
    }
}


#Preview {
    ContentView()
}
